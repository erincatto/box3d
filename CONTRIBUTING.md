# Contributing to Box3D

This project is open source under the [MIT License](LICENSE). Please read this document
before opening a pull request — **PRs that skip these steps will be closed.**

---

## TL;DR

1. **Discuss first.** Open an issue (or a [Discussion](../../discussions)) and
   wait for it to be labeled `ready-for-pr` before writing code.
2. **Sign the CLA.** A bot will prompt you on your first PR. One signature
   covers all future contributions.
3. **Link the issue.** Put `Closes #123` in your PR description. A required
   check enforces this.
4. **Follow the style.** C17, no compiler warnings, formatted with
   `clang-format`.
5. **Comments** Comment your code with human words. Create Doxygen comments for public APIs.
6. **Follow the LLM guidelines.** See the LLM guidelines in README.md

---

## 1. Discuss before you build

Open an issue or discussion before creating a PR.

I will review the issue or discussion and apply the **`ready-for-pr`** label. Please don't open
a PR until you get the label.

Please wait follow this process. I don't want people wasting time writing a PR that won't be accepted.

---

## 2. Contributor License Agreement (CLA)

Before your first contribution can be merged, you must sign the project's
**Individual Contributor License Agreement**. When you open your first pull
request, a bot will post a link; signing takes about a minute and covers all
your future contributions.

**What this means:** you retain the copyright to your contribution. The CLA
grants the project maintainer a broad, perpetual, irrevocable license to use,
modify, sublicense, and **relicense** your contribution. This lets the project
adapt its licensing in the future (for example, offering a commercial edition)
without having to track down every past contributor.

If you're contributing on behalf of an employer, make sure you have the right
to do so — some employment agreements assign your work to your employer, in
which case a Corporate CLA may be required. Contact the maintainer if you're
unsure.

By submitting a contribution you also confirm that it is your original work and
that you have the right to submit it under these terms.

---

## 3. Opening a pull request

Once your issue is labeled `ready-for-pr`:

1. **Fork** the repository and create a branch from `main`:

   ```bash
   git checkout -b fix/short-description
   ```

2. Make your change in small, focused commits.
3. **Reference the issue** in your PR description using a closing keyword:

   ```text
   Closes #(Issue Number)
   ```

   A required status check parses this and will fail if no linked, approved
   issue is found — the PR cannot be merged without it.
4. Ensure the full test suite and CI pass.
5. Fill out the PR template completely.

Keep PRs scoped to a single logical change. If you find yourself touching
unrelated things, split them into separate PRs (each with its own approved
issue).

---

## 4. Coding conventions

Source files must be in C17. Samples use C++20. Please follow existing coding
conventions and use clang format.

---

## 5. Tests and documentation

- Comment your code.
- Public API must have Doxygen comments.

---

## 6. LLM Policy

- Follow the LLM policy in README.md

---

## Questions?

Open a [Discussion](../../discussions) — that's the best place for anything not
covered here. Thanks for helping make Box3D better!
