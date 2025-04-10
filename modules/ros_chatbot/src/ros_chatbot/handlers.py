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
import os
import time
from abc import ABCMeta, abstractmethod
from datetime import date, timezone
from typing import List, Optional

import haipy.memory_manager as mm
from haipy.chat_history import ChatHistory
from haipy.parameter_server_proxy import EventListener, UserSessionContext
from haipy.scheduler import init as scheduler_init
from haipy.scheduler.schemas import (
    EmotionalContext,
    GoalContext,
    InterestContext,
    PhysiologicalContext,
)
from haipy.scheduler.schemas.enums import DriverStatus
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_openai import ChatOpenAI

from ros_chatbot.intention_manager import IntentionManager
from ros_chatbot.schemas import SceneContext

mm.init(os.environ["CLOUD_MONGO_DATABASE_URL"])
scheduler_init(os.environ["CLOUD_MONGO_DATABASE_URL"])

intention_manager = IntentionManager()


class Tune(BaseModel):
    style: str = Field(
        description="The response style can be academic, casual, decisive, for kids, serious, inquisitive, etc."
    )
    length: int = Field(description="What should be the word count for the response?")


class Plan(BaseModel):
    """Plan to follow in future"""

    steps: List[str] = Field(
        description="different steps to follow, should be in sorted order but do not include the order number"
    )


class CompletedTasks(BaseModel):
    steps: List[str] = Field(description="The tasks that have been completed")


class Attribute(BaseModel):
    attribute: str = Field(description="The attribute name")
    value: str = Field(description="The extracted attribute value")


class Attributes(BaseModel):
    attributes: List[Attribute] = Field(
        description="List of extracted attribute objects"
    )


class EventHandler(metaclass=ABCMeta):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        """Initialize the event handler.

        Args:
            session_context: Context for the current user session
            scene_context: Context for the current scene
        """
        # Set default values for trigger keys and timer interval if not defined by subclass
        self.trigger_keys = getattr(self, "trigger_keys", [])
        self.timer_interval = getattr(self, "timer_interval", None)

        self.session_context = session_context
        self.scene_context = scene_context
        self.logger = logging.getLogger(
            f"hr.ros_chatbot.handlers.{self.__class__.__name__}"
        )

    def set_event_listener(self, event_listener: EventListener):
        for key in self.trigger_keys:
            event_listener.on_key_change(key, self)
        if self.timer_interval:
            event_listener.on_timer(self.timer_interval, self)

    @property
    def input(self):
        return self.session_context.get("input", "")

    @abstractmethod
    def handle(self, event) -> bool:
        """return True if the event was handled, False otherwise"""
        pass


class LLMEventHandler(EventHandler):
    def __init__(
        self,
        session_context: UserSessionContext,
        scene_context: SceneContext,
        prompt_template: str,
    ):
        super().__init__(session_context, scene_context)
        self.prompt = ChatPromptTemplate.from_template(
            prompt_template, template_format="jinja2"
        )
        self.session_context = session_context
        self.chat_history = ChatHistory(f"{session_context.ns}.history")

    @property
    def history(self):
        self.chat_history.set_history_key(f"{self.session_context.ns}.history")
        return self.chat_history.format_history_text()


class TuneRegulator(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["situational_prime"]
        prompt = """Evaluate the situation and determine what style and length that fit best.

        The situation
        {{situational_prime}}
        """
        super(TuneRegulator, self).__init__(session_context, scene_context, prompt)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(Tune)

    def handle(self, event) -> bool:
        if "situational_prime" not in self.session_context:
            return False
        context = {"situational_prime": self.session_context["situational_prime"]}
        result = self.pipeline.invoke(context)
        if result:
            self.session_context["response_length"] = result.length
            self.session_context["response_style"] = result.style
            self.logger.warning("Update response tune %s", result)
        return True


class EmotionalHandler(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["physiological_context", "turns"]
        prompt = (
            "You are a humanoid robot with sophisticated emotional capabilities. Your emotional state is defined by several key dimensions:\n"
            "- Sadness vs Happiness (-1 to 1)\n"
            "- Excitement vs Calmness (-1 to 1)\n"
            "- Anxiety vs Confidence (-1 to 1)\n"
            "- Hostility vs Friendliness (-1 to 1)\n"
            "- Boredom vs Interest (-1 to 1)\n\n"
            "Analyze the conversation and your current physiological state to determine your emotional state along these dimensions.\n"
            "If any dimension becomes significantly negative (below -0.5), suggest specific actions to help restore emotional balance.\n\n"
            "{% if physiological_context %}\n"
            "Your current physiological state:\n"
            "{{physiological_context}}\n"
            "{% endif %}\n"
            "\n"
            "{% if history %}\n"
            "Recent conversation history:\n"
            "{{history}}\n"
            "{% endif %}\n"
            "\n"
            "Provide:\n"
            "1. A numerical assessment for each emotional dimension\n"
            "2. An overall description of your emotional state\n"
            "3. Effective behavioral reactions to restore emotional balance if needed\n"
        )
        super().__init__(session_context, scene_context, prompt)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(EmotionalContext)

    def handle(self, event) -> bool:
        context = {
            "history": self.history,
            "physiological_context": self.session_context.get("physiological_context"),
        }
        result = self.pipeline.invoke(context)
        if result:
            context = EmotionalContext(**result)
            self.session_context["emotion_context"] = str(context)
            self.logger.warning("Update emotion context %s", context)
            # fires when the emotion valence is smaller than the threshold
            if context.valence < 0.5:
                intention_manager.add_driver(context)
        return True


class PhysiologicalHandler(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["body_state"]
        prompt = (
            "You are an advanced autonomous social robot with sophisticated self-monitoring and self-repair capabilities.\n"
            "Your integrated sensor system continuously monitors:\n"
            "- Servo positions and movements\n"
            "- Component temperatures\n"
            "- System load and resource usage\n"
            "- Power levels and consumption\n"
            "- Overall system stability\n\n"
            "Analyze the following self-diagnostic data and make sure to provide:\n"
            "1. A clear assessment of your physiological state\n"
            "2. Recommended actions to take to recover from any issues\n\n"
            "Current self-diagnostic data:\n"
            "{{body_state}}\n"
        )
        super().__init__(session_context, scene_context, prompt)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.9
        ).with_structured_output(PhysiologicalContext)

    def handle(self, event) -> bool:
        if self.session_context.get("body_state"):
            context = {"body_state": self.session_context.get("body_state")}
            result = self.pipeline.invoke(context)
            if result:
                context = PhysiologicalContext(**result)
                self.session_context["physiological_context"] = str(context)
                self.logger.warning("Update physiological context %s", context)
                if context.valence < 0.5:
                    intention_manager.add_driver(context)
        return True


class Planner(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["objective"]
        prompt_template = (
            "For the given objective, come up with a simple step by step plan.\n"
            "This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps.\n"
            "The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.\n\n"
            "# Situation\n"
            "{{situation}}\n\n"
            "# Objective\n"
            "{{objective}}\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(Plan)

    def handle(self, event) -> bool:
        context = {
            "situation": self.session_context.get("situational_prime"),
            "objective": self.session_context.get("objective"),
        }
        result = self.pipeline.invoke(context)
        if result:
            self.session_context["planned_steps"] = result.steps
            self.logger.info("Planned steps %s", result.steps)

        return True


class Evaluator(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["turns"]
        prompt_template = (
            "You are a conversational AI. You have a list of tasks to cover in your "
            "conversation with the user. Based only on the conversation, list the tasks in the plan "
            "that have been considered completed.\n\n"
            "# Your plan\n"
            "{{planned_steps}}\n\n"
            "# History\n"
            "{{history}}\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(CompletedTasks)

    def handle(self, event) -> bool:
        planned_steps = "\n".join(
            [
                f"{i}. {task}"
                for i, task in enumerate(
                    self.session_context.get("planned_steps", []), 1
                )
            ]
        )
        self.chat_history.set_history_key(f"{self.session_context.ns}.history")
        if self.chat_history.is_empty():
            self.logger.warning("No history to evaluate plan")
            return True
        context = {"planned_steps": planned_steps, "history": self.history}
        if planned_steps:
            result = self.pipeline.invoke(context)
            if result:
                self.session_context["done_steps"] = (
                    self.session_context.get("done_steps", []) + result.steps
                )
                self.logger.warning("Completed steps %s", result.steps)
        return True


class Replanner(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["done_steps"]
        prompt_template = (
            "You are conversional AI.\n"
            "You are in this situation: {{situational_prime}}\n"
            "{% if body_state %}\nYour body state: {{body_state}}\n{% endif %}\n"
            "{% if emotion_context %}\nYour emotion state: {{emotion_context}}\n{% endif %}\n"
            "Your objective was: {{objective}}\n"
            "It is essential that you prioritize your needs. The needs from your physical body is the highest priority."
            "Then your emotion state is the second priority, and finally your objective is the third priority.\n\n"
            "Your original plan was:\n"
            "{{planned_steps}}\n\n"
            "You have currently done the follow tasks:\n"
            "{{done_steps}}\n\n"
            "Update your plan accordingly."
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(Plan)

    def handle(self, event) -> bool:
        planned_steps = "\n".join(
            [
                f"{i}. {task}"
                for i, task in enumerate(
                    self.session_context.get("planned_steps", []), 1
                )
            ]
        )
        done_steps = "\n".join(
            [
                f"{i}. {task}"
                for i, task in enumerate(self.session_context.get("done_steps", []), 1)
            ]
        )
        body_state = self.session_context.get("body_state")
        situational_prime = self.session_context.get("situational_prime")
        objective = self.session_context.get("objective")
        emotion_context = self.session_context.get("emotion_context")
        context = {
            "planned_steps": planned_steps,
            "done_steps": done_steps,
            "objective": objective,
            "situational_prime": situational_prime,
            "body_state": body_state,
            "emotion_context": emotion_context,
        }
        result = self.pipeline.invoke(context)
        if result:
            self.session_context["planned_steps"] = result.steps
            self.logger.warning("Replanned steps %s", result.steps)
        return True


class GoalPlanner(LLMEventHandler):
    """Create a goal based on the objective"""

    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["objective"]
        prompt_template = (
            "You are a humanoid robot with advanced goal planning capabilities. Your task is to create a detailed, achievable plan for accomplishing objectives.\n\n"
            "Current Date: {{date}}\n"
            "Objective: {{objective}}\n\n"
            "As an autonomous robot, you should:\n"
            "1. Analyze the objective carefully and identify the key components required for success\n"
            "2. Break down the objective into 3-5 concrete, sequential subgoals that build upon each other\n"
            "3. For each subgoal:\n"
            "   - Make it specific and measurable (e.g. 'Gather 3 data points' rather than 'Collect data')\n"
            "   - Consider any prerequisites or dependencies\n"
            "   - Set realistic completion criteria\n"
            "4. If the objective has time constraints:\n"
            "   - Set appropriate deadlines for the overall goal\n"
            "   - Allow buffer time for unexpected challenges\n"
            "5. Evaluate the overall goal:\n"
            "   - Importance (0-1): How critical is this to your function and purpose?\n"
            "   - Ease (0-1): How achievable is this given your capabilities?\n"
            "   - Motivation (0-1): How aligned is this with your core directives?\n\n"
            "Format your response as a GoalContext object containing:\n"
            "- Name: A concise title for the goal\n"
            "- Description: Detailed explanation of what needs to be accomplished\n"
            "- Subgoals: List of specific tasks with clear success criteria\n"
            "- Deadline: Target completion date/time if time-sensitive\n"
            "- Metrics: Numerical scores for importance, ease, and motivation\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(GoalContext)

    def handle(self, event) -> bool:
        context = {
            "objective": self.session_context.get("objective"),
            "date": date.today().isoformat(),
        }
        print(context)
        result = self.pipeline.invoke(context)
        if result:
            goal_context = GoalContext(**result)
            # convert local time to UTC
            goal_context.deadline = (
                goal_context.deadline.astimezone(timezone.utc)
                if goal_context.deadline
                else None
            )
            goal_context.status = DriverStatus.PENDING
            self.session_context["goal_context"] = goal_context
            intention_manager.add_driver(goal_context)
        return True


class ConversationGoalCreator(LLMEventHandler):
    """Create a goal based on the conversation history"""

    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["end_scene"]
        prompt_template = (
            "You are a humanoid robot tasked with planning how to achieve goals effectively.\n"
            "You are in this situation: {{situational_prime}}\n"
            "{% if history %}\n"
            "Recent conversation history:\n"
            "{{history}}\n"
            "{% endif %}\n\n"
            "Instructions:\n"
            "1. Analyze the conversation history and identify any explicit or implicit goals, needs, or commitments\n"
            "2. Create a clear, actionable goal that addresses the key points from the conversation\n"
            "3. Break down the goal into 2-4 concrete subgoals that can be tracked and measured\n"
            "4. Consider any time constraints or deadlines mentioned\n"
            "5. Rate the following on a scale of 0-1:\n"
            "   - Importance: How critical is this goal?\n"
            "   - Ease: How achievable is this goal?\n"
            "   - Motivation: How motivated are you to pursue this goal?\n\n"
            "Provide your response as a GoalContext object with:\n"
            "- A clear description of the overall goal\n"
            "- A prioritized list of subgoals\n"
            "- Deadline (if mentioned or implied)\n"
            "- Metrics for importance, ease, and motivation\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(GoalContext)

    def handle(self, event) -> bool:
        context = {
            "situational_prime": self.session_context.get("situational_prime"),
            "history": self.history,
        }
        result = self.pipeline.invoke(context)
        if result:
            goal_context = GoalContext(**result)
            goal_context.status = DriverStatus.PENDING
            self.session_context["goal_context"] = goal_context
            intention_manager.add_driver(goal_context)
        return True


class GoalEvaluator(LLMEventHandler):
    """Evaluate goals"""

    class Evaluation(BaseModel):
        goals_completed: List[int] = Field(
            description="The index of the goals that have been completed"
        )
        goals_cancelled: List[int] = Field(
            description="The index of the goals that are cancelled"
        )
        goals_dormant: List[int] = Field(
            description="The index of the goals that need to be put on hold"
        )
        goals_active: List[int] = Field(
            description="The index of the goals that are active"
        )
        reason: str = Field(
            description="The reason for the decision of goals to be completed, cancelled, dormant or active"
        )

    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.timer_interval = 5
        prompt_template = (
            "You are a humanoid robot with advanced goal planning capabilities. Your task is to evaluate the status and progress of multiple goals.\n\n"
            "Current Situation: {{situational_prime}}\n\n"
            "{% if goals %}\n"
            "Active Goals:\n"
            "{{goals}}\n"
            "{% endif %}\n\n"
            "Recent Conversation Context:\n"
            "{{history}}\n\n"
            "Instructions:\n"
            "1. Carefully analyze each goal and determine its current status:\n"
            "   - COMPLETED: Goal has been fully achieved\n"
            "   - CANCELLED: Goal is no longer relevant or achievable\n"
            "   - DORMANT: Goal should be temporarily paused\n"
            "   - ACTIVE: Goal is currently being worked on\n\n"
            "2. For each goal, consider:\n"
            "   - Progress made so far\n"
            "   - Current relevance to situation\n"
            "   - Resource availability\n"
            "   - Dependencies on other goals\n"
            "   - Time constraints\n"
            "   - Changes in context or priorities\n\n"
            "3. Provide clear reasoning for each status change, especially for:\n"
            "   - Why certain goals are considered complete\n"
            "   - Why goals need to be cancelled\n"
            "   - Why goals should become dormant\n"
            "   - Why goals should become active\n\n"
            "Return the indices of goals for each status category along with detailed reasoning.\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(GoalEvaluator.Evaluation)

    def handle(self, event) -> bool:
        drivers = intention_manager.get_drivers()
        goals = [
            f"{i}. {driver.name}: {driver.description}"
            for i, driver in enumerate(drivers)
        ]
        context = {
            "situational_prime": self.session_context.get("situational_prime"),
            "goals": goals,
            "history": self.history,
        }
        result = self.pipeline.invoke(context)
        if result:
            self.logger.warning("Goal evaluation %s", result)
            for driver in drivers:
                driver.context.status = DriverStatus.PENDING
            for index in result.goals_completed:
                driver = drivers[index]
                driver.context.status = DriverStatus.COMPLETED
            for index in result.goals_cancelled:
                driver = drivers[index]
                driver.context.status = DriverStatus.CANCELLED
            for index in result.goals_dormant:
                driver = drivers[index]
                driver.context.status = DriverStatus.DORMANT
            for index in result.goals_active:
                driver = drivers[index]
                driver.context.status = DriverStatus.ACTIVE
            # for driver in drivers:
            #    driver.save()
        return True


class ConflictResolver(LLMEventHandler):
    """Detects conflicts among goals, interests, deep drives etc, and resolve them"""

    class Resolution(BaseModel):
        chosen_goal: List[int] = Field(description="The index of the goal to be chosen")
        reason: str = Field(description="The reason for the chosen goal")

    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.timer_interval = 5
        prompt_template = (
            "You are a conflict resolution agent for a humanoid robot, responsible for prioritizing and selecting between competing goals.\n\n"
            "Key Decision Rules:\n"
            "1. Deep drives (fundamental needs/values) take highest priority\n"
            "2. Consider urgency, importance, and resource constraints\n"
            "3. Evaluate potential impact on long-term objectives\n"
            "4. Account for current emotional and physiological state\n"
            "5. Ensure chosen goals align with core robot directives\n\n"
            "Current Situation: {{situational_prime}}\n\n"
            "{% if goals %}\n"
            "Active Goals Under Consideration:\n"
            "{{goals}}\n"
            "{% endif %}\n\n"
            "Instructions:\n"
            "1. Analyze each goal's priority level and type (deep drive vs standard goal)\n"
            "2. Evaluate potential conflicts and dependencies between goals\n"
            "3. Select the most appropriate goal(s) to pursue\n"
            "4. Provide clear reasoning for your selection\n\n"
            "Which goal(s) should be prioritized and why?"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(ConflictResolver.Resolution)

    def handle(self, event) -> bool:
        drivers = intention_manager.get_drivers()
        goals = [
            f"{i}. {driver.name}: {driver.description}"
            + (
                "\n   Subgoals:\n   - "
                + "\n   - ".join(str(subgoal) for subgoal in driver.context.subgoals)
                if driver.context.subgoals
                else ""
            )
            for i, driver in enumerate(drivers)
        ]
        context = {
            "situational_prime": self.session_context.get("situational_prime"),
            "goals": goals,
        }
        result = self.pipeline.invoke(context)
        if result:
            self.logger.warning("Conflict resolution %s", result)
            for driver in drivers:
                driver.context.status = DriverStatus.PENDING
            for index in result.chosen_goal:
                driver = drivers[index]
                driver.context.status = DriverStatus.ACTIVE
            for driver in drivers:
                driver.save()
        return True


class GoalUpdator(EventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.timer_interval = 5
        super().__init__(session_context, scene_context)

    def handle(self, event) -> bool:
        goals = intention_manager.prioritize_drivers()
        goals = [
            f"[Status: {driver.context.status}, Level: {driver.level}] {driver.name}: {driver.description}"
            for i, driver in enumerate(goals)
        ]
        self.session_context["goals"] = goals
        self.logger.warning("Updated goals %s", goals)
        return True


class InterestHandler(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.timer_interval = 5
        prompt_template = (
            "You are a humanoid robot.\n"
            "You are in this situation: {{situational_prime}}\n"
            "{% if body_state %}\nYour body state: {{body_state}}\n{% endif %}\n"
            "{% if emotion_context %}\nYour emotion state: {{emotion_context}}\n{% endif %}\n"
            "{% if history %}\n"
            "The conversation\n"
            "{{history}}\n"
            "{% endif %}\n"
        )
        super().__init__(session_context, scene_context, prompt_template)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(InterestContext)

    def handle(self, event) -> bool:
        context = {
            "history": self.history,
            "situational_prime": self.session_context.get("situational_prime"),
            "body_state": self.session_context.get("body_state"),
            "emotion_context": self.session_context.get("emotion_context"),
        }
        result = self.pipeline.invoke(context)
        if result:
            context = InterestContext(**result)
            self.session_context["interest_context"] = str(context)
            self.logger.warning("Update interest state %s", context)
            if context.interest_valence > 0.5:
                intention_manager.add_driver(context)
        return True


class PersonObjectUpdator(EventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["interlocutor"]
        super(PersonObjectUpdator, self).__init__(session_context, scene_context)

    def handle(self, event) -> bool:
        """
        Handle the event to update the person objects based on the interlocutors' names.
        """
        interlocutors = self.session_context.get("interlocutor", "").split(",")
        interlocutors = [name.strip() for name in interlocutors if name.strip()]

        if not interlocutors:
            # Reset person objects
            self.session_context["person_objects"] = []
            self.scene_context.scene_document.person_ids = []
            self.scene_context.update_scene_document()
            return True

        person_objects = []
        person_object_infos = []

        for name in interlocutors:
            # Load person object
            person_object = mm.Person.load_people_object(name)
            if person_object:
                self.logger.warning("Person object loaded: %s", person_object)
            else:
                first_name, last_name = mm.Person.split_name(name)
                person_object = mm.Person(first_name=first_name, last_name=last_name)
                person_object.save()
                self.logger.warning("New person object created: %s", person_object)

            person_objects.append(person_object)

            person_object_info = mm.Person.format_person_object(person_object)
            if person_object_info:
                person_object_infos.append(person_object_info)

        if person_object_infos:
            # person_objects is used by prompt template
            self.session_context["person_objects"] = person_object_infos

        for person_object in person_objects:
            if person_object.id not in self.scene_context.scene_document.person_ids:
                self.scene_context.scene_document.person_ids.append(person_object.id)

        self.scene_context.scene_document.save()
        return True


class Diarykeeper(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["ended_scene"]  # make a diary when the scene ends
        prompt = """You are a journalist who takes notes in the form of a diary. You help analyze and maintain the personal experiences, thoughts, and events from the perspective of a robot who is part of the conversation, according to the given conversation history between the robot and a human.

The date was: {{ date }}
{% if location %}
The place the conversation took place was: {{location}}
{% endif %}
{% if interlocutor %}
The human interlocutor was: {{interlocutor}}
{% endif %}
{% if objective %}
The objective of the robot was: {{objective}}
{% endif %}

# Conversation History
{{history}}
"""
        super(Diarykeeper, self).__init__(session_context, scene_context, prompt)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(mm.DiaryData)
        self.chat_history.n_messages = None
        self.chat_history.ai_message_prompt = "Robot: "
        self.context = Diarykeeper.Context()

    class Context(BaseModel):
        uid: Optional[str]
        sid: Optional[str]
        objective: Optional[str]
        location: Optional[str]
        interlocutor: Optional[str]
        history: Optional[str]

    def set_diary_context(self, **context):
        self.context = Diarykeeper.Context(**context)

    def handle(self, event) -> bool:
        context = {
            "objective": self.context.objective,
            "history": self.context.history,
            "date": date.today(),
            "location": self.context.location,
            "interlocutor": self.context.interlocutor,
        }
        if self.context.history == ChatHistory.empty_history:
            return True
        result = self.pipeline.invoke(context)
        if result:
            self.session_context["diary"] = result
            diary = mm.Diary(
                uid=self.context.uid,
                conversation_id=self.context.sid,
                interlocutor=self.context.interlocutor,
                **result,
            )
            diary.save()
            self.logger.warning("Diary %s", diary.json())
        return True


class AttributeExtractor(LLMEventHandler):
    def __init__(
        self, session_context: UserSessionContext, scene_context: SceneContext
    ):
        self.trigger_keys = ["input"]
        prompt = """
Given the conversation history as context and the user's response below, extract the attributes in the user response and return a JSON message with those attributes. Do not make up things out of user's response.

## Conversation History
{{history}}

<user_response>
{{input}}
</user_response>

{% if attribute_set %}
attributes to extract: {{attribute_set}}. If there is no such attribute, return nothing.
{% else %}
Automatially extract the personal attributes about the users. Do not extract trival information. If there is no attributes, return nothing.
{% endif %}
"""
        super(AttributeExtractor, self).__init__(session_context, scene_context, prompt)
        self.pipeline = self.prompt | ChatOpenAI(
            model="gpt-4o", temperature=0.6
        ).with_structured_output(Attributes)

    def handle(self, event) -> bool:
        if not self.input:
            return True
        if not self.session_context.get("person_object_id"):
            return True
        history = ""
        messages = self.chat_history.filtered_messages(self.input)
        for message in reversed(messages):
            if message.type == "ai":
                history = message.content
                break
        context = {
            "input": self.input,
            "history": history,
            "attribute_set": self.session_context.get("attribute_set"),
        }
        result = self.pipeline.invoke(context)
        if result and result.attributes and "attribute_set" in self.session_context:
            del self.session_context["attribute_set"]
        self.logger.warning("Extracted attributes %s", result)

        # update interlocutor
        for attribute in result.attributes:
            if (
                attribute.attribute == "name"
                and self.session_context.get("interlocutor") != attribute.value
            ):
                self.logger.warning("Update interlocutor %s", attribute.value)
                self.session_context["interlocutor"] = attribute.value
                time.sleep(3)  # wait for Person Object to be updated

        if self.session_context.get("person_object_id"):
            person_object = mm.Person.get(self.session_context["person_object_id"])
            if person_object and result.attributes:
                person_object = person_object.run()
                # update the attributes of the person object
                for attribute in result.attributes:
                    person_object.add_attribute(attribute.attribute, attribute.value)
                person_object.save()
        return True


class handlers:
    _handler_classes = [
        AttributeExtractor,
        Diarykeeper,
        Evaluator,
        PersonObjectUpdator,
        GoalPlanner,
        PhysiologicalHandler,
        EmotionalHandler,
        Replanner,
        TuneRegulator,
    ]

    locals().update({handler.__name__: handler for handler in _handler_classes})

    @classmethod
    def get_all_handlers(cls):
        """Returns a list of all handler classes"""
        return cls._handler_classes


if __name__ == "__main__":
    import os

    logging.basicConfig(level=logging.INFO)

    # os.environ["LANGCHAIN_TRACING_V2"] = "true"
    from haipy.parameter_server_proxy import UserSessionContext

    session_context = UserSessionContext(uid="default", sid="default")
    listener = EventListener()
    listener.subscribe("default.default")
    # PhysiologicalHandler(session_context, None).set_event_listener(listener)
    # EmotionalHandler(session_context, None).set_event_listener(listener)
    GoalPlanner(session_context, None).set_event_listener(listener)
    # GoalUpdator(session_context, None).set_event_listener(listener)
    # ConflictResolver(session_context, None).set_event_listener(listener)
    # InterestHandler(session_context, None).set_event_listener(listener)
    # GoalEvaluator(session_context, None).set_event_listener(listener)
    while True:
        time.sleep(1)
