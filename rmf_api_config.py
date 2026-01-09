#!/usr/bin/env python3
# Configuration for RMF API Server
from api_server.default_config import config

# Override default settings
config.update({
    "host": "0.0.0.0",
    "port": 8000,
    "db_url": "sqlite:///home/robot1/rmf_ws/rmf_api.db",
    "public_url": "http://localhost:8000",
    "cache_directory": "/home/robot1/rmf_ws/run/cache",
    "log_level": "INFO",
    "builtin_admin": "admin",
    "jwt_public_key": None,  # Disable authentication for local development
    "oidc_url": None,
    "iss": None,  # Disable authentication
    "ros_args": [],
})
