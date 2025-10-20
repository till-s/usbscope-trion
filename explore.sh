#!/usr/bin/env bash

# This is different from the sweeper provided by efinix.
# We want the git-hash to be synthesized into the design.
# Efinix uses multiple PNR runs on the same synthesized
# design which means that the hash cannot be incorporated.
#
# Here we
#  checkout multiple subdirectories
#  create a seed-branch in each subdir
#  commit to the seed-branch
#  run the tool
# Eventually, we have timing reports from the desings
# which do have the git-hash included.

xmlname="scope_test_v2.xml"
constraints="scope_test_v2.pt.sdc"
perixmlname="scope_test_v2.peri.xml"

seeds="4 5 6 7 8 9"

mkdir -p explore
pushd explore
for i in ${seeds}; do
    if [ ! -d swipe_${i} ]; then
      git clone ../ swipe_${i}
    fi
    pushd swipe_${i}
    if ! [ -e ${constraints} ]; then
      ln -s "../../${constraints}" .
    fi
    if ! [ -e ${perixmlname} ]; then
      ln -s "../../${perixmlname}" .
    fi
    for m in ../../modules/*; do
      git config --replace-all "submodule.modules/`basename ${m}`.url" "${m}"
    done
    git submodule update --init --recursive
    sed -i -e 's/name="seed" *value="[0-9]\+"/name="seed" value="'"${i}"'"/' "${xmlname}"
    git checkout -B "seed_${i}"
    if ! git diff-index --quiet HEAD --; then
      git commit -m "Seed ${i}" ${xmlname}
    fi
    ./update_git_version_pkg.sh
    efx_run ${xmlname} &
    popd
done

