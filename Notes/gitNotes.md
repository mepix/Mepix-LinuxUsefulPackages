# Using Git

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

```

## Branches

```sh
# list available branches (locally)
git branch

# fetch branches availble (remote)
get fetch

git checkout <existing_branch>

git checkout -b <new_branch>

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

## House Keeping

```sh
# Untrack Unwanted Files
git rm --cached <filename>
git commit
```

## References

- [How to Switch Branches on Git](https://devconnected.com/how-to-switch-branch-on-git/)
- [SO: Cherrypicking](https://stackoverflow.com/questions/9339429/what-does-cherry-picking-a-commit-with-git-mean)
