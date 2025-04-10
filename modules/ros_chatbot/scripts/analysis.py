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

import time

from haipy.memory_manager.memory_model import search_scenes_by_conditions
from haipy.memory_manager.scene_postprocessing import update_scene_memory_models
from haipy.scheduler.intention_manager import (
    extract_driver_metrics,
    plot_metrics,
    search_drivers,
)
from haipy.scheduler.schemas.enums import DriverStatus
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

# Create a console instance for rich output
console = Console()


def plot():
    for driver_type in [
        "GoalDriver",
        "EmotionalDriver",
        "PhysiologicalDriver",
        "InterestDriver",
        "DeepDriver",
    ]:
        drivers = search_drivers(driver_type=driver_type)
        metrics_df = extract_driver_metrics(drivers)
        console.print(f"[bold blue]Metrics for {driver_type}:[/bold blue]")
        console.print(metrics_df)
        plot_metrics(driver_type, metrics_df)

    time.sleep(1000)


def test(driver_type: str, lookback_hours: int = 72):
    console.print(
        f"[bold green]Searching for drivers of type: [/bold green][yellow]{driver_type}[/yellow]"
    )
    drivers = search_drivers(
        driver_type=driver_type,
        lookback_hours=lookback_hours,
    )
    metrics_df = extract_driver_metrics(drivers)
    console.print(
        Panel(str(metrics_df), title=f"Metrics for {driver_type}", border_style="cyan")
    )

    for driver in drivers:
        panel_content = (
            f"[bold]ID:[/bold] [cyan]{driver.id}[/cyan]\n"
            f"[bold]Created at:[/bold] [cyan]{driver.created_at}[/cyan]\n"
            f"[bold]Updated at:[/bold] [cyan]{driver.updated_at}[/cyan]\n"
            f"[bold]Type:[/bold] [magenta]{driver.type}[/magenta]\n"
            f"[bold]Conversation IDs:[/bold] [yellow]{driver.conversation_ids}[/yellow]\n"
            f"[bold]Metrics:[/bold] {driver.context.metrics.model_dump()}\n"
            f"[bold]Valence:[/bold] [green]{driver.context.valence}[/green]\n"
            f"[bold]Status:[/bold] [red] {driver.context.status}[/red]\n"
            f"[bold]Name:[/bold] [blue]{driver.name}[/blue]"
        )

        if driver.type == "GoalDriver":
            panel_content += (
                f"\n[bold]Deadline:[/bold] [yellow]{driver.context.deadline}[/yellow]"
            )

        console.print(
            Panel(
                panel_content,
                title="Driver Info",
                border_style="green",
            )
        )

        if driver.conversation_ids:
            i = 0
            for conversation_id in driver.conversation_ids:
                scenes = search_scenes_by_conditions(conversation_id=conversation_id)
                if scenes:
                    console.print("[bold purple]Chat History:[/bold purple]")
                    for scene in scenes:
                        if not scene.chat_history.messages:
                            update_scene_memory_models([scene])
                        if scene.chat_history.messages:
                            console.print(
                                Panel(
                                    "\n".join(
                                        [
                                            f"[{'green' if message.role == 'ai' else 'blue' if message.role == 'human' else 'white'}]{message.role.title()}: {message.text}[/]"
                                            for message in scene.chat_history.messages
                                        ]
                                    ),
                                    title=f"Scene {i+1}",
                                    border_style="yellow",
                                )
                            )
                            i += 1


def test2():
    console.print("[bold green]Searching for ACTIVE and PENDING drivers[/bold green]")
    drivers = search_drivers(statuses=[DriverStatus.ACTIVE, DriverStatus.PENDING])

    table = Table(title="Active and Pending Drivers")
    table.add_column("Created At", style="cyan")
    table.add_column("Type", style="magenta")
    table.add_column("Conversation IDs", style="yellow")
    table.add_column("Valence", style="green")
    table.add_column("Name", style="blue")

    for driver in drivers:
        table.add_row(
            str(driver.created_at),
            driver.type,
            ", ".join(driver.conversation_ids),
            str(driver.context.valence),
            driver.name,
        )

    console.print(table)


if __name__ == "__main__":
    import os

    from haipy.memory_manager import init as mm_init
    from haipy.scheduler import init

    init(os.environ["CLOUD_MONGO_DATABASE_URL"])
    mm_init(os.environ["CLOUD_MONGO_DATABASE_URL"])
    # test("EmotionalDriver")
    test("GoalDriver", lookback_hours=None)
    # test2()
