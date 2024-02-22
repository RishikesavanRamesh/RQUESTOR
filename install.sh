#!/bin/bash

# Script to deploy a self-hosted service using docker-compose.

# generate a secret using /dev/random as randomness source
getSecret() {
  length=$([[ -z $1 ]] && echo 24 || echo $1)
  cat /dev/random | od -An -x -N 1024 | tr -d '\n ' | cut -c -$length
}


# Check if the script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi



# Uninstall conflicting Docker versions
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do 
    apt-get remove -y $pkg
done

# Add Docker's official GPG key
apt-get update
apt-get install ca-certificates curl
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  tee /etc/apt/sources.list.d/docker.list > /dev/null

apt-get update

# Install Docker components
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin















BOT_NAME=QUESTBOT

SBOT_NAME=$(echo ${BOT_NAME} | tr '[:lower:]' '[:upper:]')









DIR=~/$BOT_NAME
# We need to create some folders to ensure the right owner including the
# generated certs (see certs/generate.sh).
mkdir -p $DIR/certs

# We need to delete any existing DB or else the admin password at the end
# will not match what's already in the DB.
if [[ -e $DIR/db ]]; then
  echo "There already exists an installation in $DIR. Please remove $DIR/db and try again."
  exit 3;
fi;

mkdir -p $DIR/db

# For dev: create a folder read by cloud-app where capabilities can be updated live
mkdir -p /tmp/caps

cd $DIR

# Generate .env file
cat << EOF > $DIR/.env


VPN_CONFIG_FILE_LOCATION=

# In dev we want to start some auxiliary services in docker-compose. Set to prod
# in production.
COMPOSE_PROFILES=dev   ## dev || debug , prod

EOF

echo $DIR/.env written


# Write docker-compose.yaml file (without interpreting variables)
cat << 'EOF' > $DIR/docker-compose.yaml
version: "3.9"

SEVICES INCLUDE VPN CONTAINER AND BOT CONTAINER:deploy 

EOF




# Write docker-compose.yaml file (without interpreting variables)
cat << 'EOF' > /etc/systemd/system/${SBOT_NAME}-system.service
[Unit]
Description=$Robot's Docker Compose Service
Requires=docker.service
After=docker.service

[Service]
Restart=always
WorkingDirectory=$DIR/
ExecStart=/usr/bin/docker-compose up --remove-orphans
ExecStop=/usr/bin/docker-compose down

[Install]
WantedBy=multi-user.target

EOF

systemctl daemon-reload
systemctl enable ${SBOT_NAME}-system.service
systemctl start ${SBOT_NAME}-system.service





# thsi file is written ( yeah , with many typos ) for the purpose of easying our deployment of dyno atman, in a fresh jetson nano 

# 1) this file expects the source code,( compiled binaries of sourcecode to be provided within the dockerimage from the private repo )
# i) this also implies that there is a need to generate unique identity keys for every deployment so that it can be used to pull the updated docker image from our private docker repo repo


# 2) also the default user in the jetson name host system, and the dyno container is the root ()  this may be changed to a specific user to constrain the permission leaks , like ros or dyno  <-username