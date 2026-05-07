#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../frontend"
npm run build
cp -r dist/. ../backend/static/
echo "Done — frontend built and deployed to backend/static"
