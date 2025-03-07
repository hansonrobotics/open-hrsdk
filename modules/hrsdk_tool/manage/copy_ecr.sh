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

# Copy repositories from one region to another.
#
# Usage:
# ./copy_ecr.sh
#

set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source_region=ap-east-1 # Hong Kong
target_region=ap-south-1 # Mumbai

create_repos() {
    local repos=$(aws --region $source_region ecr describe-repositories | jq -r '.repositories | .[] | .repositoryName')
    for repo in ${repos[@]}; do
        aws ecr create-repository --repository-name $repo --region $target_region && echo "created $repo in $target_region"
    done
}

push_images() {
    local target_image source_image
    source $BASEDIR/../common.sh

    # load user config
    user_config_file=$USER_CONFIG_DIR/config.sh
    if [[ -f $user_config_file ]]; then
        source $user_config_file
    else
        error "SDK was not configured. Please run \"hrsdk init\""
    fi
    info "HRSDK_VERSION=$HRSDK_VERSION"

    export VERSION_FILE=$USER_CONFIG_DIR/versions/$HRSDK_VERSION
    export AWS_REGION_NAME=${target_region}
    export ECR_URL=222132024866.dkr.ecr.${target_region}.amazonaws.com

    aws s3 cp --region=$AWS_REGION_NAME --only-show-errors \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/versions/${HRSDK_VERSION} \
        ${VERSION_FILE} || fail "ERROR: Couldn't get version info"

    login_ecr

    while read -r line; do
        [[ $line =~ ^#.* ]] && continue
        repo=$(echo $line | cut -d= -f2)

        ECR_URL=222132024866.dkr.ecr.${source_region}.amazonaws.com
        source_image=$ECR_URL/$repo # replace the ECR_URL
        ECR_URL=222132024866.dkr.ecr.${target_region}.amazonaws.com
        target_image=$ECR_URL/$repo # replace the ECR_URL

        info "docker tag $source_image $target_image"
        docker tag $source_image $target_image
        info "docker push $target_image"
        docker push $target_image
    done < ${VERSION_FILE}
}

push_images
