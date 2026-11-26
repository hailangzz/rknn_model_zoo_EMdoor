#!/bin/bash

set -u

PROJECT_DIR="/home/robot/share/CarpetSegmentProject"
SESSION_NAME="ai_debug"
START_SCRIPT="${PROJECT_DIR}/start_ai_debug.sh"

cd "${PROJECT_DIR}" || exit 1

echo "========================================"
echo "AI Debug Launcher Started"
echo "Project: ${PROJECT_DIR}"
echo "Session: ${SESSION_NAME}"
echo "========================================"

while true; do

    if /usr/bin/tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then

        echo "$(date '+%Y-%m-%d %H:%M:%S') - ai_debug session is running."

    else

        echo "$(date '+%Y-%m-%d %H:%M:%S') - ai_debug session not found."
        echo "$(date '+%Y-%m-%d %H:%M:%S') - Creating ai_debug session..."

        /usr/bin/tmux new-session \
            -d \
            -s "${SESSION_NAME}" \
            /bin/bash "${START_SCRIPT}"

        if [ $? -eq 0 ]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S') - ai_debug session created successfully."
        else
            echo "$(date '+%Y-%m-%d %H:%M:%S') - Failed to create ai_debug session."
        fi

    fi

    sleep 5

done