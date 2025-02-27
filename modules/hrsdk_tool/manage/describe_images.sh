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

# describes ECR docker images
#

set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/../config.sh

show_help() {
cat << EOF

Describe the cloud docker images.

Usage: $0 --region region --repo repository:tag [--repo repository:tag ...]
  --region
    AWS region name
  --repo
    repository name in the form of "name:tag"

Example:
    $0 --region ap-east-1 --repo hansonrobotics/soultalk:dev --repo hansonrobotics/ros-perception:v1.2

EOF
}

describe_image() {
    local region=$1
    shift
    for repo in $@; do
        local name=$(echo $repo | cut -d: -f1)
        local tag=$(echo $repo | cut -d: -f2)
        aws ecr --region $region describe-images --image-ids "imageTag=$tag" --repository-name $name \
            | jq -r '.imageDetails | .[] | .repositoryName, .imageDigest, .imageSizeInBytes' | awk -v tag=$tag 'BEGIN {RS=""} {printf "%-40s\t%-10s\t%-80s\t%-20s\n",$1,tag,$2,$3}'
    done
}

while [[ $# > 0 ]];
do
    case "$1" in
        --region)
            region=$2
            shift
            shift
            ;;
        --repo)
            shopt -s nocasematch
            if [[ $2 =~ [a-z0-9_.-]+:[a-z0-9_.-]+ ]]; then
                repo+=($2)
            else
                error "format of --repo must be repository:tag"
                exit 1
            fi
            shift
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            error "Unknown argument $1"
            show_help
            exit 1
            ;;
    esac
done

if [[ -z $region ]]; then
    error "--region was missing"
    show_help
    exit 1
fi
if [[ -z $repo ]]; then
    error "--repo was missing"
    show_help
    exit 1
fi

printf "%-40s\t%-10s\t%-80s\t%-20s\n" REPOSITORY TAG DIGEST SIZE
describe_image $region ${repo[@]}
