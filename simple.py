# Use Ubuntu as the base image
FROM ubuntu:latest

# Update and install necessary packages
RUN apt-get update && \
    apt-get install -y python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install virtualenv
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install virtualenv

# Create a virtual environment and install dependencies
RUN virtualenv $VIRTUAL_ENV
COPY requirements.txt /app/requirements.txt
RUN $VIRTUAL_ENV/bin/pip install -r /app/requirements.txt

# Grant access to the virtual environment to the user apla1
RUN chown -R apla1:apla1 $VIRTUAL_ENV

# Set the working directory
WORKDIR /app

# Switch to the user apla1
USER apla1

# Run your application
CMD ["python3", "app.py"]
