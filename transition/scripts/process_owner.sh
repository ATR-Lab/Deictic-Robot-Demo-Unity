#!/usr/bin/env bash
# Source into a foreground owner. Only the sessions created here are signaled.
pids=()
pid_graces=()
owned_group_live() {
    # Reap exited leaders promptly; zombies alone are not running children.
    ps -eo pgid=,stat= | awk -v group="$1" '$1 == group && $2 !~ /^Z/ { live=1 } END { exit !live }'
}
start_owned() {
    start_owned_with_grace 8 "$@"
}
start_owned_with_grace() {
    local grace=$1
    shift
    [[ "$grace" =~ ^[0-9]+$ ]] && ((grace>=1 && grace<=60)) || { echo 'Invalid owned cleanup grace' >&2; return 2; }
    /usr/bin/python3 -c 'import os,signal,sys
os.setsid()
signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' "$@" &
    pids+=("$!")
    pid_graces+=("$grace")
}
cleanup_owned() {
    local index pid deadline
    # Reverse order lets the console revoke/drain while its simulator still runs.
    for ((index=${#pids[@]}-1;index>=0;index--)); do
        pid=${pids[index]}
        # If cleanup races the tiny pre-setsid bootstrap, TERM that owned PID;
        # inherited shell SIGINT may still be ignored until bootstrap resets it.
        kill -INT -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        deadline=$((SECONDS+pid_graces[index]))
        while owned_group_live "$pid" && ((SECONDS<deadline)); do sleep .1; done
        if owned_group_live "$pid"; then
            kill -TERM -- "-$pid" 2>/dev/null || true
            deadline=$((SECONDS+2))
            while owned_group_live "$pid" && ((SECONDS<deadline)); do sleep .1; done
        fi
        if owned_group_live "$pid"; then kill -KILL -- "-$pid" 2>/dev/null || true; fi
        wait "$pid" 2>/dev/null || true
    done
    pids=()
    pid_graces=()
}
