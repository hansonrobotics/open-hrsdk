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

Package the configs in the path.

Usage: $0 <config path>

Example:
    $0 ~/hansonrobotics/hrsdk_configs

EOF
}

package_configs() {
    local dir=/tmp/hrsdk_configs
    local archive=/tmp/latest.tar
    local config_path=$1
    local branch=$(git -C ${config_path} rev-parse --abbrev-ref HEAD)
    local tag=$(git -C ${config_path} describe --tags --exact-match || echo "")
    if [[ ! -z $tag ]]; then
        branch=$tag
    fi

    branch=${branch//\//-}  # replace / with -
    if [[ $branch == 'models' ]]; then
        fail "\"models\" is a reserved config name"
    fi
    if [[ $branch == 'data' ]]; then
        fail "\"data\" is a reserved config name"
    fi
    get_confirm_warn "Are you sure to upload the configs to \"$branch\"? [y/n] "
    [[ ${confirm} -eq 0 ]] && exit

    info "Archiving ${config_path}..."
    git -C ${config_path} archive --prefix ${branch}/ -o ${archive} HEAD

    info "Decompressing archive..."
    [[ -d ${dir} ]] && rm -r ${dir}
    mkdir -p $dir && tar xf ${archive} -C ${dir}

    # write the tag file for record
    local tag_file=${dir}/${branch}/tag
    echo "config name: $branch" >${tag_file}
    echo "config hash: $(git -C ${config_path} rev-parse $branch)" >>${tag_file}
    echo "created time: $(date)" >>${tag_file}

    local folders_to_compress=(configs)
    # compress folders
    cd ${dir}/${branch}
    for folder in ${folders_to_compress[@]}; do
        [[ ! -d $folder ]] && continue
        tar=${folder%/}.tar.gz
        info "Compressing $folder..."
        GZIP=-n tar zcf $tar $folder && rm -r $folder  # GZIP=-n: no gzip timestamp in the tarball
    done

    $BASEDIR/s3/sync.sh ${dir}/${branch}

    # upload the files that get ignored becaues they do not change size
    AWS_REGION_NAME=${AWS_REGION_NAME:-ap-east-1}
    AWS_PROFILE=${AWS_PROFILE:-hrsdk_admin}

    COPY="aws s3 cp --region=$AWS_REGION_NAME --profile $AWS_PROFILE"
    BUCKET=s3://dl.hrsdk.hansonrobotics.com
    # tag file
    $COPY ${tag_file} ${BUCKET}/${branch}/
    # voice license
    for file in ${dir}/${branch}/voices/*.lic; do
        $COPY $file ${BUCKET}/${branch}/voices/
    done
}

if [[ $# > 0 ]]; then
    package_configs "$1"
else
    show_help
    exit 1
fi

