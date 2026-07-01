#!/bin/bash
set -e

echo "=== Updating apt packages ==="
apt-get update

echo "=== Installing system tools ==="
apt-get install -y \
  git \
  openssh-client \
  build-essential \
  gfortran \
  libopenblas-dev \
  liblapack-dev \
  pkg-config \
  curl \
  unzip \
  ca-certificates

echo "=== Installing Oh My Posh ==="
curl -s https://ohmyposh.dev/install.sh | bash -s -- -d /usr/local/bin

echo "=== Configuring Oh My Posh for bash ==="
sed -i '/oh-my-posh init bash/d' /root/.bashrc
echo 'eval "$(oh-my-posh init bash --config /workspaces/field/.devcontainer/mytheme.omp.json)"' >> /root/.bashrc

echo "=== Checking Python version ==="
python --version

echo "=== Installing Python packaging tools compatible with Python 3.6 ==="
python -m pip install --upgrade \
  "pip<22" \
  "setuptools<60" \
  wheel

echo "=== Installing Python requirements ==="
pip install -r .devcontainer/requirements.txt

echo "=== Installing multiple-view-intelligence from GitHub ==="
pip install \
  --no-deps \
  --ignore-requires-python \
  --no-build-isolation \
  "git+ssh://git@github.com/natuition/multiple-view-intelligence.git@v0.1.6#egg=multiple-view-intelligence"

echo "=== Done ==="