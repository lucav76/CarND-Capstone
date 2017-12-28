#!/usr/bin/env bash

session="ros"

# IF `ros` session is already running kill it
if [[ $(tmux list-session | grep -i $session) ]]; then
    echo "[info] Found $session. Killing it"
    tmux kill-session -t $session
else
    echo "[info] No $session is found.."
fi

if [[ ! -f "../simulator" ]]; then
    echo "[warn] simulator is not found. Please create a symlink ../simulator"
    echo "============================="
    echo "ln -s your_simulator_path/linux_sys_int/sys_int.x86_64 ../simulator"
    exit 1
fi

# begin tmux background
tmux new-session -s $session -d -n bash
tmux send-keys "source devel/setup.bash" C-m

# window-1 run roslaunch
tmux new-window -t $session -n core
tmux send-keys "source devel/setup.bash && roslaunch launch/styx.launch" C-m

# window-2 run simulator
tmux new-window -t $session -n simulator
tmux send-keys "source devel/setup.bash && ../simulator" C-m

# Attach the session
tmux attach -t $session
