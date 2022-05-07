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
- [Splitting a subfolder out into a new repository](https://docs.github.com/en/get-started/using-git/splitting-a-subfolder-out-into-a-new-repository)
