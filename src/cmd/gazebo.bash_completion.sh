#!/usr/bin/env bash

# bash tab-completion

# This is a per-library function definition, used in conjunction with the
# top-level entry point in ign-tools.

# NOTE: In Garden+, replace `gazebo` in function name to align with subcommand,
# for ign-tools/etc/ign.bash_completion.sh to find this function.
function _gz_gazebo
{
  if [[ ${COMP_WORDS[COMP_CWORD]} == -* ]]; then
    # Specify options (-*) word list for this subcommand
    COMPREPLY=($(compgen -W "
      -g
      --iterations
      --levels
      --network-role
      --network-secondaries
      --record
      --record-path
      --record-resources
      --record-topic
      --log-overwrite
      --log-compress
      --playback
      -r
      -s
      -v --verbose
      --gui-config
      --physics-engine
      --render-engine
      --render-engine-gui
      --render-engine-server
      --version
      -z
      -h --help
      --force-version
      --versions
      " -- "${COMP_WORDS[COMP_CWORD]}" ))
    return
  else
    # Just use bash default auto-complete, because we never have two
    # subcommands in the same line. If that is ever needed, change here to
    # detect subsequent subcommands
    COMPREPLY=($(compgen -o default -- "${COMP_WORDS[COMP_CWORD]}"))
    return
  fi
}
