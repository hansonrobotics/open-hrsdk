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

# Push the files to ECR and S3
#

show_help() {
cat << EOF

Usage: hrsdk push [--user-data]

    --user-data
        push user data to the cloud

EOF
}

parse_args() {
    while [[ $# > 0 ]]; do
        case "$1" in
            --user-data)
                export PUSH_USER_DATA=1
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                error "Unknown option $1"
                show_help
                exit 1
                ;;
        esac
    done
}

_upload_storage() {
    generate_version_file ${VERSION_FILE}
    source $VERSION_FILE
    $LAUNCHER $LAUNCHER_ARGS run --rm storage
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    set -e

    source $(dirname $(readlink -f ${BASH_SOURCE[0]}))/config.sh
    update_env

    check_version
    parse_args $@

    if [[ $PUSH_USER_DATA == 1 ]]; then
        _upload_storage
    fi
    info "Finished"
fi
