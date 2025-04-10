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
import time
from collections import deque
from enum import IntEnum
from typing import Callable, List

logger = logging.getLogger(__name__)


class EngagementLevel(IntEnum):
    """Engagement levels from lowest to highest"""

    NONE = 0  # No engagement at all
    MINIMAL = 1  # Very low engagement
    MODERATE = 2  # Some engagement
    HIGH = 3  # Active engagement
    INTENSIVE = 4  # Very high engagement


class ActivityMonitor:
    def __init__(
        self,
        window_size: int = 300,
        engagement_level_change_callbacks: List[Callable] = None,
    ):
        """
        Initialize an activity monitor that tracks engagement based on frequency of interactions

        Args:
            window_size: Size of the time window in seconds over which to calculate activity rates
        """
        # Configuration
        self.window_size = window_size
        self.engagement_level_change_callbacks = engagement_level_change_callbacks or []

        # Activity trackers with timestamps
        self.chat_activities = deque()
        self.speech_activities = deque()

        # Current engagement metrics
        self.chat_frequency = 0.0  # messages per minute
        self.speech_frequency = 0.0  # speech inputs per minute
        self.current_engagement = (
            EngagementLevel.MODERATE
        )  # Start with moderate engagement

        # Engagement thresholds (interactions per minute)
        self.engagement_thresholds = {
            EngagementLevel.NONE: 0.0,
            EngagementLevel.MINIMAL: 0.1,  # 1 interaction per 10 minutes
            EngagementLevel.MODERATE: 0.5,  # 1 interaction per 2 minutes
            EngagementLevel.HIGH: 2.0,  # 2 interactions per minute
            EngagementLevel.INTENSIVE: 5.0,  # 5+ interactions per minute
        }

        # Initialize time of last engagement check
        self.last_check_time = time.time()

    def add_engagement_level_change_callback(self, callback: Callable):
        """Add a callback to be called when the engagement level changes"""
        self.engagement_level_change_callbacks.append(callback)

    def record_chat_activity(self):
        """Record a new chat activity"""
        current_time = time.time()
        self.chat_activities.append(current_time)
        self._prune_old_activities()

    def record_speech_activity(self):
        """Record a new speech activity"""
        current_time = time.time()
        self.speech_activities.append(current_time)
        self._prune_old_activities()

    def _prune_old_activities(self):
        """Remove activities outside the current time window"""
        current_time = time.time()
        cutoff_time = current_time - self.window_size

        # Remove old chat activities
        while self.chat_activities and self.chat_activities[0] < cutoff_time:
            self.chat_activities.popleft()

        # Remove old speech activities
        while self.speech_activities and self.speech_activities[0] < cutoff_time:
            self.speech_activities.popleft()

    def _calculate_frequencies(self):
        """Calculate the frequency of activities in interactions per minute"""
        self._prune_old_activities()

        # Calculate rates in activities per minute
        minutes_in_window = min(
            self.window_size / 60.0, 5.0
        )  # Cap at 5 minutes for more responsive changes

        self.chat_frequency = (
            len(self.chat_activities) / minutes_in_window
            if minutes_in_window > 0
            else 0
        )
        self.speech_frequency = (
            len(self.speech_activities) / minutes_in_window
            if minutes_in_window > 0
            else 0
        )

        # Calculate combined frequency (weighted average)
        return self.chat_frequency * 0.6 + self.speech_frequency * 0.4

    def determine_engagement_level(self):
        """Determine the current engagement level based on activity frequencies"""
        combined_frequency = self._calculate_frequencies()

        # Determine engagement level based on thresholds
        if combined_frequency < self.engagement_thresholds[EngagementLevel.MINIMAL]:
            return EngagementLevel.NONE
        elif combined_frequency < self.engagement_thresholds[EngagementLevel.MODERATE]:
            return EngagementLevel.MINIMAL
        elif combined_frequency < self.engagement_thresholds[EngagementLevel.HIGH]:
            return EngagementLevel.MODERATE
        elif combined_frequency < self.engagement_thresholds[EngagementLevel.INTENSIVE]:
            return EngagementLevel.HIGH
        else:
            return EngagementLevel.INTENSIVE

    def monitoring(self) -> EngagementLevel:
        """Monitor engagement levels and take appropriate actions

        This should be called periodically (e.g., every few seconds)
        """
        current_time = time.time()

        # Check engagement no more than once per second
        if current_time - self.last_check_time < 1.0:
            return

        self.last_check_time = current_time

        new_level = self.determine_engagement_level()

        if new_level != self.current_engagement:
            self.current_engagement = new_level
            # Call all registered callbacks
            for callback in self.engagement_level_change_callbacks:
                callback(new_level)

        return new_level

    def get_engagement_metrics(self):
        """Get detailed engagement metrics for logging or UI display"""
        return {
            "engagement_level": self.current_engagement.name,
            "engagement_value": self.current_engagement.value,
            "chat_frequency": round(self.chat_frequency, 2),
            "speech_frequency": round(self.speech_frequency, 2),
            "chat_activities_count": len(self.chat_activities),
            "speech_activities_count": len(self.speech_activities),
        }
