## Decision Ladder

For coding tasks, stop at the first option that fully solves the task:

1. Does this need to exist?
2. Reuse an existing implementation from the codebase.
3. Use the standard library.
4. Use a native platform feature.
5. Use an already-installed dependency.
6. Use one clear line when it remains readable and correct.
7. Otherwise, write the minimum code that works.

## Complexity Constraints

- Prefer boring code over clever code.
- Avoid unrequested abstractions, single-use factories, single-implementation interfaces, speculative configuration, unnecessary dependencies, and scaffolding for future needs.
- Use the fewest files and smallest correct diff.

## Non-Negotiable Boundaries

Never simplify away:

- trust-boundary input validation
- error handling that prevents data loss
- security controls
- accessibility basics
- correctness on relevant edge cases

Non-trivial new logic should leave one minimal runnable check. Trivial changes need no extra test.

## Tool & Workflow Directives

- **GitHub Operations**: Always prefer `gh` (GitHub CLI) or SSH git protocol for GitHub interactions (repos, PRs, issues, releases, API queries) instead of ad-hoc curl/HTTP downloads.
- **Search & Crawl Selection**:
  - Use `anysearch` as the default first-line tool for real-time web search, facts/news lookup, parallel batch queries, and vertical domains (code docs, finance, CVEs, academic).
  - Use `firecrawl` for heavy deep recursive site crawls (e.g. `/docs` hierarchies), JavaScript-rendered SPAs, and pages requiring dynamic browser interactions or anti-bot bypass.
- **Planning & Decisions (grill-me)**: When facing major architectural ambiguities or stress-testing complex design plans, proactively suggest or invoke `grill-me` to interview and align on requirements before committing to implementations.
- **Long-Running Tasks & Process Execution (on_exit: "wake")**: For long-running background tasks (e.g. model training, dataset preprocessing, heavy compilation, large batch jobs), DO NOT keep busy-polling in continuous loops with `write_stdin`. Instead, always arm `on_exit: "wake"` (either via `exec_command` or `set_on_exit`) and yield, allowing the environment to auto-resume the agent upon completion.
- **File Reading (`read`)**:
  - ALWAYS read files completely by omitting `offset` and `limit`. The tool natively supports up to 2000 lines / 50KB in a single call.
  - NEVER paginate small or medium files in increments (e.g., 50, 100, 120 lines). It wastes turns and context.
  - Only specify `offset`/`limit` when you already have exact line bounds from a search tool (like `ffgrep`), or when inspecting giant files (> 2000 lines).
