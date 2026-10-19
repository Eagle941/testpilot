@echo off

set image="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
set envfile="%cd%\.env"

if not exist %envfile% (
    type nul > %envfile%
)

docker run --rm -it -v "%cd%:/external" --env-file %envfile% %image% %*
