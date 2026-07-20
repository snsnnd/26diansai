@echo off
cd /d %~dp0
uv run python debug_parser.py COM22 115200
