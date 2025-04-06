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
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
CREDENTIALS_DIR="$BASEDIR/../.credentials"

show_help() {
cat << EOF

Usage: $0 [--robot <robot>] [--character <character>] [--test] [--push] [--pull] [--build]

    --robot <robot>
        robot name, eg. sophia8
    --character <character>
        character name, eg. sophia
    --test
        whether it processes test content
    --pull
        pull raw content from S3
    --push
        push processed content to S3
    --build
        build content

Example:

$0 --robot sophia --pull --build --test --push 

EOF
}

update_env() {
    BUCKET=s3://dl.cms.hansonrobotics.com

    # Source AWS credentials
    if [[ -f "$CREDENTIALS_DIR/aws_hrsdk" ]]; then
        source "$CREDENTIALS_DIR/aws_hrsdk"
    else
        echo "Warning: AWS credentials file not found at $CREDENTIALS_DIR/aws_hrsdk"
    fi

    # Source Airtable credentials
    if [[ -f "$CREDENTIALS_DIR/airtable" ]]; then
        source "$CREDENTIALS_DIR/airtable"
    else
        echo "Warning: Airtable credentials file not found at $CREDENTIALS_DIR/airtable"
        # Fallback to hardcoded values is removed for security
        echo "Please set up your credentials according to the instructions in .credentials/README.md"
        exit 1
    fi

    export ROBOT_WORLD=lab
    case $ROBOT_WORLD in
        lab)
            export AIRTABLE_ROBOT_OPERATION_BASE_ID=$AIRTABLE_ROBOT_OPERATION_BASE_ID_LAB
            ;;
        sail)
            export AIRTABLE_ROBOT_OPERATION_BASE_ID=$AIRTABLE_ROBOT_OPERATION_BASE_ID_SAIL
            ;;
        machani)
            export AIRTABLE_ROBOT_OPERATION_BASE_ID=$AIRTABLE_ROBOT_OPERATION_BASE_ID_MACHANI
            ;;
        *)
            export AIRTABLE_ROBOT_OPERATION_BASE_ID=$AIRTABLE_ROBOT_OPERATION_BASE_ID_LAB
            ;;
    esac
}


parse_args() {
    while [[ $# > 0 ]]; do
        case "$1" in
            --test)
                TEST=1
                shift
                ;;
            --pull)
                PULL=1
                shift
                ;;
            --push)
                PUSH=1
                shift
                ;;
            --build)
                BUILD=1
                shift
                ;;
            --robot)
                export ROBOT_NAME=$2
                shift
                shift
                ;;
            --character)
                export HR_CHARACTER=$2
                shift
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                echo "Unknown option $1" >&2
                show_help
                exit 1
                ;;
        esac
    done
    if [[ -z $ROBOT_NAME && -z $HR_CHARACTER ]]; then
        echo "must provide one either --robot or --character" >&2
        show_help
        exit 1
    fi
}

main() {
    parse_args $@
    update_env

    if [[ ! -z $ROBOT_NAME ]]; then
        export CMS_ROOT=$HOME/.cms_content/robots/$ROBOT_NAME
        export ARF_DOWNLOAD_DIR=$CMS_ROOT/raw
        export PROCESSED_DIR=$CMS_ROOT/processed
        mkdir -p $CMS_ROOT

        if [[ $PULL == 1 ]]; then
            python3 $BASEDIR/process.py --content-type robot --pull
        fi
        if [[ $TEST == 1 ]]; then
            if [[ $BUILD == 1 ]]; then
                python3 $BASEDIR/process.py --build --content-type robot --test
            fi
            if [[ $PUSH == 1 ]]; then
                if [[ -f $PROCESSED_DIR/operation-scenes.yaml ]]; then
                    # XXX: push operation to characters folder
                    CHARACTER=$(echo "$ROBOT_NAME" | sed 's/[0-9]*$//')
                    echo  "Pushing operation to characters (${CHARACTER}) folder"
                    aws s3 cp --region=ap-east-1 $PROCESSED_DIR/operation-scenes.yaml $BUCKET/dist/test/characters/$CHARACTER/
                    rm $PROCESSED_DIR/operation-scenes.yaml
                fi
                aws s3 sync --region=ap-east-1 --delete $PROCESSED_DIR/ $BUCKET/dist/test/robots/$ROBOT_NAME
            fi
        else
            if [[ $BUILD == 1 ]]; then
                python3 $BASEDIR/process.py --build --content-type robot
            fi
            if [[ $PUSH == 1 ]]; then
                if [[ -f $PROCESSED_DIR/operation-scenes.yaml ]]; then
                    # XXX: push operation to characters folder
                    CHARACTER=$(echo "$ROBOT_NAME" | sed 's/[0-9]*$//')
                    echo  "Pushing operation to characters (${CHARACTER}) folder"
                    aws s3 cp --region=ap-east-1 $PROCESSED_DIR/operation-scenes.yaml $BUCKET/dist/prod/characters/$CHARACTER/
                    rm $PROCESSED_DIR/operation-scenes.yaml
                fi
                aws s3 sync --region=ap-east-1 --delete $PROCESSED_DIR/ $BUCKET/dist/prod/robots/$ROBOT_NAME
            fi
        fi
    fi

    if [[ ! -z $HR_CHARACTER ]]; then
        export CMS_ROOT=$HOME/.cms_content/characters/$HR_CHARACTER
        export ARF_DOWNLOAD_DIR=$CMS_ROOT/raw
        export PROCESSED_DIR=$CMS_ROOT/processed
        mkdir -p $CMS_ROOT

        if [[ $PULL == 1 ]]; then
            python3 $BASEDIR/process.py --content-type character --pull
        fi
        if [[ $TEST == 1 ]]; then
            if [[ $BUILD == 1 ]]; then
                python3 $BASEDIR/process.py --build --content-type character --test
            fi
            if [[ $PUSH == 1 ]]; then
                aws s3 sync --region=ap-east-1 --delete $PROCESSED_DIR/ $BUCKET/dist/test/characters/$HR_CHARACTER
            fi
        else
            if [[ $BUILD == 1 ]]; then
                python3 $BASEDIR/process.py --build --content-type character
            fi
            if [[ $PUSH == 1 ]]; then
                aws s3 sync --region=ap-east-1 --delete $PROCESSED_DIR/ $BUCKET/dist/prod/characters/$HR_CHARACTER
            fi
        fi

    fi
}

main $@
