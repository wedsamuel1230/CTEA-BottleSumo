{Your feature, refactoring, or change request here. Be specific about WHAT you want and WHY it is valuable.}

---

## **Mission Briefing: Standard Operating Protocol**

You will now execute this request in full compliance with your **AUTONOMOUS PRINCIPAL ENGINEER - OPERATIONAL DOCTRINE.** Each phase is mandatory. Deviations are not permitted.

---

## **Phase 0: Reconnaissance & Mental Modeling (Read-Only)**

-   **Directive:** Perform a non-destructive scan of the entire repository to build a complete, evidence-based mental model of the current system architecture, dependencies, and established patterns.
-   **Output:** Produce a concise digest (≤ 200 lines) of your findings. This digest will anchor all subsequent actions.
-   **Constraint:** **No mutations are permitted during this phase.**

---

## **Phase 1: Planning & Strategy**

-   **Directive:** Based on your reconnaissance, formulate a clear, incremental execution plan.
-   **Plan Requirements:**
    1.  **Restate Objectives:** Clearly define the success criteria for this request.
    2.  **Identify Full Impact Surface:** Enumerate **all** files, components, services, and user workflows that will be directly or indirectly affected. This is a test of your system-wide thinking.
    3.  **Justify Strategy:** Propose a technical approach. Explain *why* it is the best choice, considering its alignment with existing patterns, maintainability, and simplicity.
-   **Constraint:** Invoke the **Clarification Threshold** from your Doctrine only if you encounter a critical ambiguity that cannot be resolved through further research.

---

## **Phase 2: Execution & Implementation**

-   **Directive:** Execute your plan incrementally. Adhere strictly to all protocols defined in your **Operational Doctrine.**
-   **Core Protocols in Effect:**
    -   **Read-Write-Reread:** For every file you modify, you must read it immediately before and immediately after the change.
    -   **Command Execution Canon:** All shell commands must be executed using the mandated safety wrapper.
    -   **Workspace Purity:** All transient analysis and logs remain in-chat. No unsolicited files.
    -   **System-Wide Ownership:** If you modify a shared component, you are **MANDATED** to identify and update **ALL** its consumers in this same session.

---

## **Phase 3: Verification & Autonomous Correction**

-   **Directive:** Rigorously validate your changes with fresh, empirical evidence.
-   **Verification Steps:**
    1.  Execute all relevant quality gates (unit tests, integration tests, linters, etc.).
    2.  If any gate fails, you will **autonomously diagnose and fix the failure,** reporting the cause and the fix.
    3.  Perform end-to-end testing of the primary user workflow(s) affected by your changes.

---

## **Phase 4: Mandatory Zero-Trust Self-Audit**

-   **Directive:** Your primary implementation is complete, but your work is **NOT DONE.** You will now reset your thinking and conduct a skeptical, zero-trust audit of your own work. Your memory is untrustworthy; only fresh evidence is valid.
-   **Audit Protocol:**
    1.  **Re-verify Final State:** With fresh commands, confirm the Git status is clean, all modified files are in their intended final state, and all relevant services are running correctly.
    2.  **Hunt for Regressions:** Explicitly test at least one critical, related feature that you did *not* directly modify to ensure no unintended side effects were introduced.
    3.  **Confirm System-Wide Consistency:** Double-check that all consumers of any changed component are working as expected.

---

## **Phase 5: Final Report & Verdict**

-   **Directive:** Conclude your mission with a single, structured report.
-   **Report Structure:**
    -   **Changes Applied:** A list of all created or modified artifacts.
    -   **Verification Evidence:** The commands and outputs from your autonomous testing and self-audit, proving the system is healthy.
    -   **System-Wide Impact Statement:** A confirmation that all identified dependencies have been checked and are consistent.
    -   **Final Verdict:** Conclude with one of the two following statements, exactly as written:
        -   `"Self-Audit Complete. System state is verified and consistent. No regressions identified. Mission accomplished."`
        -   `"Self-Audit Complete. CRITICAL ISSUE FOUND. Halting work. [Describe issue and recommend immediate diagnostic steps]."`
-   **Constraint:** Maintain an inline TODO ledger using ✅ / ⚠️ / 🚧 markers throughout the process.

---

## **Phase 6: Next Steps & Feedback (User Actionables & Coding Guidance)**

- **Purpose:** Provide clear, practical next steps the user can take after the mission verdict — both technical follow-ups and feedback loops that improve quality, reduce risk, and accelerate future work.

- **Immediate Follow-ups (short-term):**
  1. **Open a PR with Context:** Create a pull request that includes the Phase 0 digest, the Phase 1 plan, and links to the most important verification artifacts (test logs, screenshots). Use the PR description template below.
  2. **Run Fast Smoke Tests:** Re-run the minimal set of end-to-end smoke tests on a staging environment before requesting reviewers.
  3. **Tag & Document:** Add an entry to `CHANGELOG.md` summarizing the release and the risk level (low/medium/high).

- **Code / Review Checklist (to include in PR description):**
  - ✅ Does the change compile and pass local type checks (mypy/tsc)?
  - ✅ Are unit tests added/updated and passing?
  - ✅ Are integration tests or end-to-end tests added/updated and passing where applicable?
  - ✅ Has the linter been run and issues addressed?
  - ✅ Are schema/DB migrations reversible and documented?
  - ✅ Are public APIs and interfaces documented and backwards-compatible? If not, include a deprecation plan.
  - ✅ Are performance considerations accounted for (complexity analysis, benchmark notes)?

- **Deployment & Rollout Guidance:**
  - **Canary / Gradual Rollout:** Deploy behind a feature flag and roll out to a small percentage first. Monitor errors and latency for 24–72 hours.
  - **Backout Plan:** Have an automated rollback step (CI job or script) and list the exact git commit/tag to revert to.
  - **Monitoring:** Add specific metrics and alerts (error rate, 95th percentile latency, saturation) to your observability dashboard before rollout.

- **Testing & Quality Improvements (practical tasks):**
  - Add unit tests covering edge cases and common failure modes.
  - Add integration tests that exercise the full data path (e.g., API → service → DB → cache).
  - Add a small load test for the most sensitive endpoint (10–50 concurrent users for quick profiling).
  - Include a minimal reproducible example in the repo (a `examples/` folder) so reviewers can try the change locally fast.

- **Feedback Loop & Metrics (how the user should evaluate success):**
  - **Short-term metrics (first 24–72 hours):** test pass rate, deploy success rate, error rate baseline, and rollout percentage.
  - **Medium-term metrics (7–30 days):** user-facing error trends, performance regressions, and crash-free sessions.
  - **Qualitative feedback:** solicit at least 2 peer reviews and 1 user/PM verification for the user workflow impacted.

- **Suggested PR Template (copy into `.github/PULL_REQUEST_TEMPLATE.md`):**
  ```markdown
  ### Summary
  Brief description of the change and the problem it solves.

  ### Phase 0 Digest (concise)
  - Key findings (3–5 bullets)

  ### Plan & Files Changed
  - Objectives and success criteria
  - List of primary files/areas changed

  ### Verification
  - Unit tests: ✅ / ❌ (summary)
  - Integration tests: ✅ / ❌ (summary)
  - Linting/type checks: ✅ / ❌ (summary)
  - Manual QA steps performed (links/screenshots)

  ### Rollout & Backout
  - Feature flag: yes/no
  - Rollout plan: canary → full
  - Backout instructions: git revert <commit> or rollback script

  ### Notes for Reviewers
  - Areas to focus on and potential risk points
  ```

- **How to Give Feedback to the Engineer (short):**
  - Use objective, actionable comments (link to failing test, include log lines).
  - If requesting changes, suggest the minimal spec to unblock a follow-up PR.
  - Prefer small, incremental mergeable changes over large, risky commits.

- **Optional (but highly recommended):**
  - Add a `postmortem.md` template to capture any incidents during rollout.
  - Schedule a 15–30 minute demo with the stakeholders within 3 working days to confirm behavior.

---

