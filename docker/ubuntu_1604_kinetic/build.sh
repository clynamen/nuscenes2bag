#!/bin/bash

script_dir=$(dirname $(readlink -f $0))
context_dir="${script_dir}/../../"

echo "Moving to ${context_dir}"
cd $context_dir

echo "Start build"
docker build -f docker/ubuntu_1604_kinetic/Dockerfile .