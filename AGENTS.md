# Repository instructions

Use `uv` for Python work in this repository. Sync the locked environment with `uv sync --locked` and run Python commands or scripts with `uv run --locked`. Use `uv add` to change dependencies and commit the updated `uv.lock`. Do not use bare `pip` or the system Python for project work.

For Python changes, run `uv run --locked ruff check .`, `uv run --locked ruff format --check .`, `uv run --locked ty check --error-on-warning`, and `uv run --locked python -m unittest discover -s tests -p 'test_*.py'`.

Use Context7 MCP for current documentation whenever a task asks about a library, framework, SDK, API, CLI tool, or cloud service. Start with `resolve-library-id`, then use `query-docs` for the relevant concept. Prefer Context7 over web search for library documentation. This does not apply to refactoring, scripts written from scratch, business-logic debugging, code review, or general programming concepts.
