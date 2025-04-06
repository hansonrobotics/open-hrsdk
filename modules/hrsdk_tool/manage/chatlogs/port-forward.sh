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
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
CREDENTIALS_DIR="$SCRIPT_DIR/../../.credentials"
KUBE_CONFIG="$CREDENTIALS_DIR/kube_config"

if ! hash kubectl ; then
    curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
    sudo mv kubectl /usr/local/bin/kubectl
fi

# Check if kube_config exists
if [ ! -f "$KUBE_CONFIG" ]; then
    echo "Error: Kubernetes configuration file not found at $KUBE_CONFIG"
    echo "Please set up your credentials according to the instructions in .credentials/README.md"
    exit 1
fi

echo -n "Token: "
read -s token
echo "Connecting..."
kubectl port-forward -n hr --token $token --kubeconfig "$KUBE_CONFIG" services/mysql 30306:3306

