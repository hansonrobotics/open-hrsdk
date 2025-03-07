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

# package the configs
#

set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/../common.sh

show_help() {
cat << EOF

Upload the performances in the configs to s3

Usage: $0 <performance path>

Example:
    $0 ~/hansonrobotics/hrsdk_configs/performances

EOF
}

BUCKET=s3://dl.cms.hansonrobotics.com
AWS_REGION_NAME=${AWS_REGION_NAME:-ap-east-1}
AWS_PROFILE=${AWS_PROFILE:-hrsdk_admin}

SYNC="aws s3 sync --region=$AWS_REGION_NAME --no-follow-symlinks --profile $AWS_PROFILE --size-only"

upload() {
    local performance_root_dir=$1
    local tmp_dir=/tmp/performances
    local timeline_package
    mkdir -p $tmp_dir

    info $(date)
    info "Preparing"
    pushd $performance_root_dir >/dev/null
    for folder in $(find . -maxdepth 2 -mindepth 2 -type d); do
        if [[ -f $folder/.cmscopy ]]; then
            # timeline folder with .cmscopy in it is considered as a copy from s3
            warn "Ignore $folder"
            continue
        fi
        robot_name=$(echo "$folder" | cut -d/ -f2)
        timeline_name=$(echo "$folder" | cut -d/ -f3)
        pushd $robot_name >/dev/null
        timeline_package=${timeline_name%/}.tar.gz
        timeline_package=$(echo $timeline_package | tr '[:upper:]' '[:lower:]')  # filename lowercase
        GZIP=-n tar zcf ${tmp_dir}/${timeline_package} $timeline_name
        popd >/dev/null
    done
    popd >/dev/null

    info "Syncing"
    $SYNC ${tmp_dir} ${BUCKET}/performances/${folder_name}
    rm -r $tmp_dir
    info "Completed"
}

if [[ $# > 0 ]]; then
    upload "$1"
else
    show_help
    exit 1
fi

