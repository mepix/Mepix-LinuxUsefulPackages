# Using Git

## Configuration

You need to tell `git` who you are before you can `push` or `pull`!

```sh
git config --global user.name "FIRST_NAME LAST_NAME"
git config --global user.email "MY_NAME@example.com"
```

**Note:** omitting the `--global` flag will set the configurations for the current, local repository.

## Basics

```sh
git pull
git push
git log
```

## Commits

```sh
# Check Your Current Tree
git status

# Add a file to the commit
git add

# Add a message to the commit
git commit -m "this is your commit message"

# Selectively Grabbing Commit
git switch <desired_branch_to_add_commit>
git cherry-pick -x <commit_hash>

# Squashing Commits:
# 1. Get the current number of commits (X)
git status
# 2. Begin an interactive (-i) rebase where X is the num of previous commits to squash
git rebase -i HEAD~X
# 3. After squashing, edit the last commit message 
git commit --amend
```

## Undoing

```sh
# Deletes everything and throws away previous commit!
git reset --hard HEAD~1

# Undo commit, but keep changes (unstaged)
git reset HEAD~1

# Undo commit and keep changes (staged)
git reset --soft HEAD~1
```

## Branches

```sh
# list available branches (locally)
git branch

# fetch branches availble (remote)
get fetch

git checkout <existing_branch>

git checkout -b <new_branch>

# push local branch to remote
git push -u origin <new_branch>

# Create Branch with a specific commit
git checkout -b <new_branch> <commit_number>

# Deleting Branches
git branch -D <local_branch_name>
git push origin --delete <remote_branch_name>
```

## Merging

```sh
# To Merge A into B
git checkout <B>
git merge <A>

# Resolving Conflicts (Recommended to install meld)
git status
git mergetool
git commit
```

## Rebasing

### Basic Commands

```sh
# Commands:
# p, pick <commit> = use commit
# r, reword <commit> = use commit, but edit the commit message
# e, edit <commit> = use commit, but stop for amending
# s, squash <commit> = use commit, but meld into previous commit
# f, fixup <commit> = like "squash", but discard this commit's log message
# x, exec <command> = run command (the rest of the line) using shell
# b, break = stop here (continue rebase later with 'git rebase --continue')
# d, drop <commit> = remove commit
# l, label <label> = label current HEAD with a name
# t, reset <label> = reset HEAD to a label
# m, merge [-C <commit> | -c <commit>] <label> [# <oneline>]
# .       create a merge commit using the original merge commit's
# .       message (or the oneline, if no original merge commit was
# .       specified). Use -c <commit> to reword the commit message.
#
# These lines can be re-ordered; they are executed from top to bottom.
#
# If you remove a line here THAT COMMIT WILL BE LOST.
#
# However, if you remove everything, the rebase will be aborted.
#
# Note that empty commits are commented out
```

### Updating History

This is useful when trying to create a clean, mostly linear commit history

```sh
# Switch to master and get latest updates
git checkout master

# Get latest updates
git pull

# Switch to development branch
git checkout feature-branch

# Perform the rebase operation
git rebase -i master
```

After this is complete, the history on the `feature-branch` will be updated before the merge

**Note:** `rebase` can also change the order of commits by reordering the listed commits.

### Splitting Commits

Perform a rebase and `edit` the committ to split

```sh
# Perform a rebase on the X previous commits
git rebase -i HEAD~X
# and select edit for the commit you want to change
```

Now, we can reset this commit and restage the files

```sh
git reset HEAD~
git add <desired-files>
git commit -m "<commit message>"
git rebase --continue
```

**Note:** Doing this will not preserve the commit message. If you want to keep the same commit message, you will need to use `git commit --reuse-message=<hash>` or `git commit -C`

### Undoing a Rebase

Look in the git reflog and then select the desired head state to return your local branch to.

```sh
# Look for the proper podition
git reflog

# Perform the reset
git reset --hard HEAD@{2}
```

**Caution:** this is `--hard` and will have consequences for unstaged changes and downstream commits!

## House Keeping

```sh
# Untrack Unwanted Files
git rm --cached <filename>
git commit
```

## Moving a subfolder to a new Repo

1. `mkdir merge_delme` and `cd merge_delme`

2. Clone the repository that contains the subfolder.

3. `cd REPOSITORY-NAME`

4. Filter out the desired subfolders (`FOLDER-NAME#`) from the rest of the files in the repository, run `git filter-repo`, :

```sh
$ git filter-repo --path FOLDER-NAME1/ --path FOLDER-NAME2/
```

5. Create a new repository on GitHub and set the new upstream:

```sh
git remote set-url origin https://github.com/USERNAME/NEW-REPOSITORY-NAME.git
```

6. Verify that the remote URL has changed with your new repository name: `git remote -v`

7. Push the directory to the new repository at the specified branch:

```sh
git push -u origin BRANCH-NAME
```

## Listing Recent Branches

This [snippet](https://stackoverflow.com/questions/5188320/how-can-i-get-a-list-of-git-branches-ordered-by-most-recent-commit) can be added to the `~/.gitconfig` to enable an alias that will list the most recent 20 branches.

```sh
[alias]
    # ATTENTION: All aliases prefixed with ! run in /bin/sh make sure you use sh syntax, not bash/zsh or whatever
    recentb = "!r() { refbranch=$1 count=$2; git for-each-ref --sort=-committerdate refs/heads --format='%(refname:short)|%(HEAD)%(color:yellow)%(refname:short)|%(color:bold green)%(committerdate:relative)|%(color:blue)%(subject)|%(color:magenta)%(authorname)%(color:reset)' --color=always --count=${count:-20} | while read line; do branch=$(echo \"$line\" | awk 'BEGIN { FS = \"|\" }; { print $1 }' | tr -d '*'); ahead=$(git rev-list --count \"${refbranch:-origin/master}..${branch}\"); behind=$(git rev-list --count \"${branch}..${refbranch:-origin/master}\"); colorline=$(echo \"$line\" | sed 's/^[^|]*|//'); echo \"$ahead|$behind|$colorline\" | awk -F'|' -vOFS='|' '{$5=substr($5,1,70)}1' ; done | ( echo \"ahead|behind||branch|lastcommit|message|author\\n\" && cat) | column -ts'|';}; r"
```

### Dependencies

#### Adding Git Filter Repo

```sh
python3 -m pip install --user git-filter-repo
```

#### Newer Version of Git

A newer version of git may be required than what is availble in the standard repository.

```sh
sudo add-apt-repository ppa:git-core/ppa -y
sudo apt-get update
sudo apt-get install git -y
git --version
```

## References

- [How to Switch Branches on Git](https://devconnected.com/how-to-switch-branch-on-git/)
- [SO: Cherrypicking](https://stackoverflow.com/questions/9339429/what-does-cherry-picking-a-commit-with-git-mean)
- [Splitting a subfolder out into a new repository](https://docs.github.com/en/get-started/using-git/splitting-a-subfolder-out-into-a-new-repository)
- [Rewritting Commit History](https://www.atlassian.com/git/tutorials/rewriting-history)
- [SO: Squashing Commits](https://stackoverflow.com/questions/5189560/how-do-i-squash-my-last-n-commits-together)
- [Amending Commits on PR](https://www.burntfen.com/2015-10-30/how-to-amend-a-commit-on-a-github-pull-request)
- [Oh Shit Git!](https://ohshitgit.com/)
- [SO: Undoing Commits](https://stackoverflow.com/questions/927358/how-do-i-undo-the-most-recent-local-commits-in-git)
- [Reorder Commits with Rebase](https://gitready.com/advanced/2009/03/20/reorder-commits-with-rebase.html)
- [SO: Undoing a Rebase](https://stackoverflow.com/questions/134882/undoing-a-git-rebase)
