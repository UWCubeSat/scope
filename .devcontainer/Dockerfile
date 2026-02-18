FROM ubuntu:latest
    
# Copy the install script into the container
COPY install.sh /tmp/install.sh

# Run the script with loggin on and remove it in a single layer
RUN chmod +x /tmp/install.sh && /tmp/install.sh || (cat /tmp/install.log && exit 1) && \
    rm -f /tmp/install.sh /tmp/install.log && \
    rm -rf /var/lib/apt/lists/*
    