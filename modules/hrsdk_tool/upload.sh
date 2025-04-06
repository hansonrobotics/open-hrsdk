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

# Upload files to S3
#

set -e
show_help() {
cat << EOF

Usage:

    hrsdk upload file1 [file2] [file3] ...
        Upload files to S3

    hrsdk upload --logs
        Upload HRSDK logs to S3

EOF
}

parse_args() {
    if [[ $# == 0 ]]; then
        show_help
        exit 0
    fi
    while [[ $# > 0 ]]; do
        case "$1" in
            -h|--help)
                show_help
                exit 0
                ;;
            --logs)
                UPLOAD_SDK_LOGS=1
                shift
                ;;
            --sync)
                SYNC_LOGS=1
                shift
                ;;
            *)
                _upload_files $@
                exit 0
                ;;
        esac
    done
}

_upload_sdk_logs() {
    local hrsdk_home_dir=$(docker volume inspect hrsdk_home | jq -r ".[0].Mountpoint")
    if [[ $SYNC_LOGS == 1 ]]; then
        echo "Syncing logs"
        sudo -E env "PATH=$PATH" aws s3 sync --region $region $hrsdk_home_dir/log s3://$bucket/$host_id/log
    else
        local dir=$(date +%Y-%m-%d-%H%M%S)
        if [[ -d /tmp/$dir ]]; then
            rm -r /tmp/$dir
        fi
        mkdir /tmp/$dir
        if sudo [ -d $hrsdk_home_dir/log ]; then
            local run_id=$(basename $(sudo readlink -m $hrsdk_home_dir/log/latest))
            if sudo [ -d $hrsdk_home_dir/log/$run_id ]; then
                sudo tar zcf /tmp/$dir/${run_id}.tar.gz -C $hrsdk_home_dir/log $run_id
            fi
        fi
        for log_file in action-orchestrator.log chatbot-server.log cog_server.log \
                dialog_act_server.log dynamixel_states.log example.log \
                feetech_data.log filebeat.log nlp-server.log qa-server.log \
                soultalk.log storage.log storage_sync.log test.log \
                timeline-generator.log webui_err.log webui_out.log; do
            if sudo [ -f $hrsdk_home_dir/log/$log_file ]; then
                sudo cp $hrsdk_home_dir/log/$log_file /tmp/$dir
            fi
        done
        sudo -E env "PATH=$PATH" aws s3 cp --recursive --region $region /tmp/$dir s3://$bucket/$host_id/$dir
        sudo rm -r /tmp/$dir
    fi
}

_upload_files() {
    for file in $@; do
        aws s3 cp --region $region $file s3://$bucket/$host_id/$file
    done
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    set -e
    source $(dirname $(readlink -f ${BASH_SOURCE[0]}))/config.sh
    region=ap-east-1
    bucket=files.hrsdk.hansonrobotics.com
    host_id=$(whoami)_$(hostname)
    parse_args $@

    if [[ $UPLOAD_SDK_LOGS == 1 ]]; then
        _upload_sdk_logs
    fi
fi
