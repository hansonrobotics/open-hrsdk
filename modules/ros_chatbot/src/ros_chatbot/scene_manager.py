##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

import logging
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError, as_completed
from typing import Dict

import haipy.memory_manager as mm
from haipy.atlas_vectordb import AtlasVectorDB
from haipy.memory_manager.memory_model import MemoryModel, search_scenes_by_conditions
from haipy.parameter_server_proxy import UserSessionContext

from ros_chatbot.schemas import Scene, SceneContext

logger = logging.getLogger("hr.ros_chatbot.scene_manager")


class SceneManager:
    """A handler for managing scene contexts and related documents."""

    def __init__(
        self,
        character: str,
        scenes: Dict[str, Scene],
        session_context: UserSessionContext,
        agent_config: dict,
        document_manager: mm.DocumentManager,
    ):
        self.character = character
        self.session_context = session_context
        self.agent_config = agent_config
        self.scenes = scenes
        self.scene_context = SceneContext(
            scenes=self.scenes, document_manager=document_manager
        )
        self._scene_document_lock = threading.Lock()
        self._diary_vector_store = AtlasVectorDB(
            db_name="scheduler_db",
            collection_name="Diary",
            vector_search_index="diary_index",
            text_key="data",
        )
        self._summary_vector_store = AtlasVectorDB(
            db_name="scheduler_db",
            collection_name="summary",
            vector_search_index="summary_index",
            text_key="summary",
        )

    def update_scenes(self, scenes: Dict[str, Scene]):
        self.scenes.update(scenes)
        self.scene_context.scenes.update(scenes)

    def update_scene(self, scene_name: str):
        """
        Update the scene context for a given scene name.

        Args:
            scene_name: Name of the scene to transition to
        """
        if not scene_name or scene_name == "auto":
            return

        logger.debug("Available scenes: %s", list(self.scene_context.scenes.keys()))

        if scene_name not in self.scene_context.scenes:
            logger.warning(
                "Scene '%s' not found in available scenes: %s",
                scene_name,
                list(self.scene_context.scenes.keys()),
            )
            return

        scene = self.scene_context.scenes[scene_name]
        self._update_vector_store(scene)
        self._update_scene_document(scene)

    def _update_vector_store(self, scene: Scene) -> None:
        """
        Update the vector store with the scene's knowledge base.

        Args:
            scene: Scene object containing the knowledge base
        """
        if not scene.knowledge_base:
            logger.debug("No knowledge base found for scene: %s", scene.name)
            return

        if self.scene_context.vector_store is None:
            self.scene_context.vector_store = AtlasVectorDB(
                db_name="memory",
                collection_name="documents",
                pre_filter={"namespace": {"$eq": scene.name}},
            )

        try:
            self.scene_context.vector_store.add_documents([scene.knowledge_base])
            logger.info(
                "Successfully loaded knowledge base for scene '%s' (%d chars)",
                scene.name,
                len(scene.knowledge_base),
            )
        except Exception as e:
            logger.error("Failed to update vector store: %s", str(e))

    def _update_scene_document(self, scene: Scene) -> None:
        """
        Update or create a SceneDocument for the given Scene.

        Args:
            scene: Scene object containing metadata and variables
        """
        try:
            with self._scene_document_lock:
                scene_doc = mm.SceneDocument.find_one(
                    {"name": scene.name, "conversation_id": self.session_context.sid}
                ).run()

                if not scene_doc:
                    scene_doc = mm.SceneDocument(
                        name=scene.name, conversation_id=self.session_context.sid
                    )
                    logger.warning("Created new scene document for: %s", scene.name)

                # Update core scene metadata
                scene_doc.metadata.update(scene.variables)
                scene_doc.metadata["knowledge_base"] = scene.knowledge_base
                scene_doc.character = scene.character

                # Add location context if available
                for location_key in ["location", "geo_location"]:
                    if location_value := self.session_context.get(location_key):
                        scene_doc.metadata[location_key] = location_value

                self.scene_context.scene_document = scene_doc
                scene_doc.save()
                logger.info("Successfully updated scene document: %s", scene.name)

        except Exception as e:
            logger.error("Failed to update scene document: %s", str(e))

    def load_relevant_scenes(self, scene_name: str) -> None:
        """
        Load the most relevant scenes that match the given scene name and location and person ids.
        If found, store its conversation summary in the session context and add to vector store.

        Args:
            scene_name: Name of the scene to search for and use as vector store namespace
        """
        if not scene_name or scene_name == "auto":
            return

        self.scene_context.memory_models = set()
        logger.warning("Loading relevant scenes for scene %s", scene_name)

        def add_memory_models(scenes):
            for scene in scenes:
                logger.warning("Adding scene %s (%s)", scene.name, scene.id)
                # skip if it's the current scene
                if scene.conversation_id == self.session_context.sid:
                    continue
                try:
                    memory_model = MemoryModel(scene)
                    memory_model.build_fields()
                    self.scene_context.memory_models.add(memory_model)
                except Exception as e:
                    logger.error(
                        f"Failed to build memory model for scene {scene.name}: {e}"
                    )

        # Get scenes matching person IDs
        with self._scene_document_lock:
            if self.scene_context:
                person_ids = self.scene_context.scene_document.person_ids
                if person_ids:
                    logger.warning("Getting person based scenes %s", person_ids)
                    person_based_scenes = search_scenes_by_conditions(
                        person_ids=person_ids, chat_history_required=True
                    )
                    logger.warning(
                        "Found %s person based scenes", len(person_based_scenes)
                    )
                    add_memory_models(person_based_scenes)
            else:
                logger.warning("No scene document found")

        # Get scenes matching location
        location = self.session_context.get("location")
        if location:
            logger.warning("Getting location based scenes %s", location)
            location_based_scenes = search_scenes_by_conditions(
                location=location,
                chat_history_required=True,
            )
            logger.warning("Found %s location based scenes", len(location_based_scenes))
            add_memory_models(location_based_scenes)

        if not self.scene_context.memory_models:
            logger.warning("No relevant scenes found")
            return

        try:
            summaries = []
            documents = []

            # Sort models by scene date
            sorted_models = sorted(
                self.scene_context.memory_models,
                key=lambda m: m.scene.created_at,
            )

            for model in sorted_models:
                if model.conversation_summary:
                    summaries.append(model.conversation_summary.to_string())
                    documents.append(
                        {
                            "text": model.conversation_summary.summary,
                            "metadata": model.conversation_summary.model_dump(),
                        }
                    )
            if summaries:
                logger.warning("Found %s past conversation summaries", len(summaries))
                self.session_context[
                    "past_conversation_summaries"
                ] = "\n------------\n".join(summaries)

            # Initialize and populate vector store
            if documents:
                if self.scene_context.vector_store is None:
                    self.scene_context.vector_store = AtlasVectorDB(
                        db_name="memory",
                        collection_name="documents",
                        pre_filter={"namespace": {"$eq": scene_name}},
                    )
                ids = self.scene_context.vector_store.add_documents(documents)
                logger.warning(f"Added {len(ids)} documents to vector store")

        except Exception as e:
            logger.error(f"Error processing conversation summaries: {e}")

    def update_last_scene_document(self):
        with self._scene_document_lock:
            if self.scene_context.scene_document:
                logger.warning(
                    "Updating last scene document %s",
                    self.scene_context.scene_document.name,
                )
                MemoryModel(self.scene_context.scene_document).build_fields()

    def create_rag(self, query):
        """
        Create RAG for the given query.
        """
        del self.session_context["rag"]

        if query.lower().startswith("event.") or query.lower().startswith(":"):
            return

        def search_diary():
            if self.character == "Sophia":
                docs = self._diary_vector_store.similarity_search(
                    query, score_threshold=0.25
                )
                logger.info("Diary search results %s", docs)
                if docs:
                    self.session_context["diary_rag"] = "\n".join(
                        f"- Timestamp: {doc.metadata['timestamp']}\n{doc.page_content}"
                        for doc, score in docs
                        if doc.metadata.get("timestamp")
                    )

        def search_summary():
            if self.character == "Sophia":
                docs = self._summary_vector_store.similarity_search(
                    query, score_threshold=0.25
                )
                logger.info("Summary search results %s", docs)
                if docs:
                    self.session_context["summary_rag"] = "\n".join(
                        f"- Timestamp: {doc.metadata['timestamp']}\n{doc.page_content}"
                        for doc, score in docs
                        if doc.metadata.get("timestamp")
                    )

        def search_knowledge_base():
            if self.scene_context and self.scene_context.vector_store:
                docs = self.scene_context.vector_store.similarity_search(
                    query, score_threshold=0.4
                )
                logger.info("Knowledge base search results %s", docs)
                if docs:
                    self.session_context["rag"] = "\n".join(
                        f"- {doc.page_content}" for doc, score in docs
                    )

        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = [
                executor.submit(search_diary),
                executor.submit(search_summary),
                executor.submit(search_knowledge_base),
            ]

            try:
                for future in as_completed(futures, timeout=0.5):
                    try:
                        future.result()
                    except Exception as e:
                        logger.error(f"Error in parallel RAG search: {e}")
            except TimeoutError:
                logger.warning("RAG search timed out after 1 second")
                for future in futures:
                    future.cancel()


if __name__ == "__main__":
    import time

    diary_vector_store = AtlasVectorDB(
        db_name="scheduler_db",
        collection_name="Diary",
        vector_search_index="diary_index",
        text_key="data",
    )
    while True:
        query = input("Enter a query: ")
        start_time = time.time()
        docs = diary_vector_store.similarity_search(query)
        print([score for _, score in docs])
        docs = [doc for doc, score in docs if score > 0.2]
        end_time = time.time()
        print(f"Time taken: {end_time - start_time} seconds")
        print(docs)

    # (0.43 + 0.37 + 0.34 + 0.32 + 0.46)/5 = 0.384
