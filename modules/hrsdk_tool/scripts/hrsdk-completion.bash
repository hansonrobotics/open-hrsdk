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

#
# To enable the completions either:
#  - place this file in /etc/bash_completion.d
#  or
#  - add the lines to ~/.bashrc
#    source <(hrsdk completion bash)

_hrsdk_complete() {
  local command cur prev
  local commands="start stop pull push init completion login version info update upload"

  if type -t _init_completion >/dev/null; then
    _init_completion || return
  else
    cur=${COMP_WORDS[COMP_CWORD]}
    prev=${COMP_WORDS[COMP_CWORD-1]}
  fi

  local _start_flags="-p -c -i --gui --hw --stt --local_ip --master_ip --storage --body --head --ecr --pull --project --dev"
  local _start_argument_flags="--local_ip --master_ip --body --head --service --env --group --ecr --pull --project"
  local _stop_flags="--project"
  local _init_flags="-f -y -h"
  local _pull_flags="--reset --configs --images --models --no-bootstrap --project --dev"
  local _pull_images_flags="--images --project"
  local _push_flags="--user-data"
  local _upload_flags="--logs --sync"
  local _complete_flags="bash services groups"

  local _services=$(hrsdk completion services)
  local _groups=$(hrsdk completion groups)
  local _pull_types=(content content-only)

  for (( i=1 ; i < ${COMP_CWORD} ; i++ )) ; do
    if [[ ${COMP_WORDS[i]} == -* ]] ; then
      _start_flags=$(printf '%s\n' "${_start_flags//${COMP_WORDS[i]}/}")  # remove the used flag
      _init_flags=$(printf '%s\n' "${_init_flags//${COMP_WORDS[i]}/}")  # remove the used flag
      _pull_flags=$(printf '%s\n' "${_pull_flags//${COMP_WORDS[i]}/}")  # remove the used flag
      _push_flags=$(printf '%s\n' "${_push_flags//${COMP_WORDS[i]}/}")  # remove the used flag
      _upload_flags=$(printf '%s\n' "${_upload_flags//${COMP_WORDS[i]}/}")  # remove the used flag
      continue;
    fi
    if [[ ${commands} == *${COMP_WORDS[i]}* ]] ; then
      command=${COMP_WORDS[i]}
    fi
  done
  _start_flags="$_start_flags --service --env --group"  # options used for multiple times are re-added here after the removal

  case ${command} in
    start)
      if [[ $_start_argument_flags == *"$prev"* ]]; then
        # the flag receives argument, stop here
        if [[ $prev == '--service' ]];  then
            COMPREPLY=($(compgen -W "${_services}" -- ${cur}))
        elif [[ $prev == '--group' ]];  then
            COMPREPLY=($(compgen -W "${_groups}" -- ${cur}))
        elif [[ $prev == '--pull' ]];  then
            COMPREPLY=($(compgen -W "${_pull_types}" -- ${cur}))
        else
            return
        fi
      elif [[ "--storage" == *"$prev"* ]]; then
        COMPREPLY=($(compgen -d -- ${cur}))
        if [[ ${#COMPREPLY[@]} -eq 1 ]]; then
            COMPREPLY[0]=${COMPREPLY[0]}/
        fi
      else
        COMPREPLY=($(compgen -W "${_start_flags}" -- ${cur}))
      fi
      ;;
    stop)
        COMPREPLY=($(compgen -W "${_stop_flags}" -- ${cur}))
      ;;
    init)
      COMPREPLY=($(compgen -W "${_init_flags}" -- ${cur}))
      ;;
    pull)
      if [[ $_pull_images_flags == *"$prev"* ]]; then
        if [[ $prev == '--images' ]];  then
            COMPREPLY=($(compgen -W "${_groups}" -- ${cur}))
        fi
      else
        COMPREPLY=($(compgen -W "${_pull_flags}" -- ${cur}))
      fi
      ;;
    push)
        COMPREPLY=($(compgen -W "${_push_flags}" -- ${cur}))
      ;;
    login|version)
      ;;
    completion)
        if [[ ${#COMP_WORDS[@]} > 3 ]]; then
            return
        fi
      if (( ${COMP_CWORD} == 2 )); then
        COMPREPLY=($(compgen -W "${_complete_flags}" -- ${cur}))
      fi
      ;;
    upload)
        COMPREPLY=($(compgen -df -W "${_upload_flags}" -- ${cur}))
      ;;
    *)
      COMPREPLY=($(compgen -W "${commands}" -- ${cur}))
      ;;
  esac
}

complete -o nospace -F _hrsdk_complete hrsdk
#complete -o plusdirs -F _hrsdk_complete hrsdk
