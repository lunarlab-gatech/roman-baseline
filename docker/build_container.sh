docker build -t roman_baseline \
    --build-arg USERNAME=$(whoami) \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    .
