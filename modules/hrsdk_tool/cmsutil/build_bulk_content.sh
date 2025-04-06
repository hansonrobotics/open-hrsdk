#!/usr/bin/env bash

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

set -e

show_help() {
cat << EOF

Usage: $0 character|robot|all

EOF
}

characters=(asha desi grace littlesophia mika profeinstein sophia)
robots=(asha grace grace1 grace10 grace11 grace12 grace13 grace14 grace15 grace2 grace3 grace4 grace5 grace6 grace7 grace8 grace9 hkbiosc mika sophia sophia17 sophia18 sophia21 sophia23 sophia24 sophia25 sophia38 sophia44 sophia45 sophia48 sophia49 sophia50 sophia51 sophia52 sophia53 sophia54 sophia55 sophia6 sophiaust)

pull_character() {
    for HR_CHARACTER in ${characters[@]}; do
        echo character: $HR_CHARACTER
        ./build_content.sh --character $HR_CHARACTER --pull
    done
}

pull_robot() {
    for ROBOT_NAME in ${robots[@]}; do
        echo robot: $ROBOT_NAME
        ./build_content.sh --robot $ROBOT_NAME --pull
    done
}
push_character() {
    for HR_CHARACTER in ${characters[@]}; do
        echo character: $HR_CHARACTER
        ./build_content.sh --character $HR_CHARACTER --build --push
    done
}
push_robot() {
    for ROBOT_NAME in ${robots[@]}; do
        echo robot: $ROBOT_NAME
        ./build_content.sh --robot $ROBOT_NAME --build --push
    done
}
push_character_test() {
    for HR_CHARACTER in ${characters[@]}; do
        echo character: $HR_CHARACTER
        ./build_content.sh --character $HR_CHARACTER --build --push --test
    done

}
push_robot_test() {
    for ROBOT_NAME in ${robots[@]}; do
        echo robot: $ROBOT_NAME
        ./build_content.sh --robot $ROBOT_NAME --build --push --test
    done
}

case "$1" in
    character)
        pull_character
        push_character
        push_character_test
        ;;
    robot)
        pull_robot
        push_robot
        push_robot_test
        ;;
    all)
        pull_character
        push_character
        push_character_test
        pull_robot
        push_robot
        push_robot_test
        ;;
    *)
        echo "Unknown option \"$1\""
        show_help
        ;;
esac
