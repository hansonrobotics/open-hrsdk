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

# list ECR docker images
#

set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/../common.sh

show_help() {
cat << EOF

List all the cloud docker images.

Usage: $0 --region region version_file
  --region
    AWS region name
  version_file
    the version file

Example:
    $0 --region ap-east-1 ~/.hrsdk/versions/dev
EOF
}

while [[ $# > 0 ]];
do
    case "$1" in
        --region)
            region=$2
            shift
            shift
            ;;
        *)
            if [[ -z $VERSION_FILE ]]; then
                VERSION_FILE=$1
                shift
            else
                error "Unknown argument $1"
                show_help
                exit 1
            fi
            ;;
    esac
done

if [[ -z $region ]]; then
    error "--region was missing"
    show_help
    exit 1
fi
if [[ -z $VERSION_FILE ]]; then
    error "version_file was missing"
    show_help
    exit 1
fi

if [[ ! -f $VERSION_FILE ]]; then
    error "version_file was not found"
    exit 1
fi

while read -r line; do
  [[ $line =~ ^#.* ]] && continue
  repo=$(echo $line | rev | cut -d/ -f1|rev)
  args+=(--repo hansonrobotics/$repo)
done < ${VERSION_FILE}

if [[ ! -z ${args} ]]; then
    $BASEDIR/describe_images.sh --region $region ${args[@]}
fi
