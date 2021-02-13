<!-- 
Please remove the appropriate section.
For example, if this is a new feature, remove all sections except for the "New feature" section

If this is your first time opening a PR, be sure to check the contribution guide:
https://ignitionrobotics.org/docs/all/contributing
-->

# 🦟 Bug fix

Fixes #<NUMBER>

## Summary
<!-- Describe your fix, including an explanation of how to reproduce the bug
before and after the PR.-->

## Checklist
- [ ] Signed all commits for DCO
- [ ] Added tests
- [ ] Updated documentation (as needed)
- [ ] Updated migration guide (as needed)
- [ ] `codecheck` passed (See [contributing](https://ignitionrobotics.org/docs/all/contributing#contributing-code))
- [ ] All tests passed (See [test coverage](https://ignitionrobotics.org/docs/all/contributing#test-coverage))
- [ ] While waiting for a review on your PR, please help review [another open pull request](https://github.com/pulls?q=is%3Aopen+is%3Apr+user%3Aignitionrobotics+repo%3Aosrf%2Fsdformat+archived%3Afalse+) to support the maintainers

**Note to maintainers**: Remember to use **Squash-Merge**

🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸

# 🎉 New feature

Closes #<NUMBER>

## Summary
<!--Explain changes made, the expected behavior, and provide any other additional
context (e.g., screenshots, gifs) if appropriate.-->

## Test it
<!--Explain how reviewers can test this new feature manually.-->

## Checklist
- [ ] Signed all commits for DCO
- [ ] Added tests
- [ ] Added example and/or tutorial
- [ ] Updated documentation (as needed)
- [ ] Updated migration guide (as needed)
- [ ] `codecheck` passed (See [contributing](https://ignitionrobotics.org/docs/all/contributing#contributing-code))
- [ ] All tests passed (See [test coverage](https://ignitionrobotics.org/docs/all/contributing#test-coverage))
- [ ] While waiting for a review on your PR, please help review [another open pull request](https://github.com/pulls?q=is%3Aopen+is%3Apr+user%3Aignitionrobotics+repo%3Aosrf%2Fsdformat+archived%3Afalse+) to support the maintainers

**Note to maintainers**: Remember to use **Squash-Merge**

🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸

# ➡️ Forward port

Port <FROM_BRANCH> to <TO_BRANCH>

Branch comparison: https://github.com/ignitionrobotics/ign-gazebo/compare/<TO_BRANCH>...<FROM_BRANCH>

**Note to maintainers**: Remember to **Merge** with commit (not squash-merge or rebase)

🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸🔸

<!-- For maintainers only -->

# 🎈 Release

Preparation for <X.Y.Z> release.

Comparison to <x.y.z>: https://github.com/ignitionrobotics/ign-gazebo/compare/<LATEST_TAG_BRANCH>...<RELEASE_BRANCH>

<!-- Add links to PRs that require this release (if needed) -->
Needed by <PR(s)>

## Checklist
- [ ] Asked team if this is a good time for a release
- [ ] There are no changes to be ported from the previous major version
- [ ] No PRs targeted at this major version are close to getting in
- [ ] Bumped minor for new features, patch for bug fixes
- [ ] Updated changelog
- [ ] Updated migration guide (as needed)
- [ ] Link to PR updating dependency versions in appropriate repository in [ignition-release](https://github.com/ignition-release) (as needed): <LINK>

<!-- Please refer to http://github.com/docs/release.md#triggering-a-release for more information -->

**Note to maintainers**: Remember to use **Squash-Merge**
