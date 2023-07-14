FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt-get update && apt-get install -y \
  python3-venv \
  bash-completion \
  && rm -rf /var/lib/apt/lists/*

USER vscode
RUN python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)" \
  && sudo $HOME/.platformio/penv/bin/pio system completion install bash --path /etc/bash_completion.d/pio \
  && echo 'export PATH=$PATH:$HOME/.platformio/penv/bin' >> ~/.bashrc

RUN $HOME/.platformio/penv/bin/pip install jinja2 intelhex future prettytable six jsonschema
