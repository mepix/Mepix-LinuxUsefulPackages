# grep

Maybe the most useful Linux utility for finding the lost arch of the covenant.

## Searching Files in a Directory for a String

```sh
# Search all files within the somewhere directory
grep -rnw '/path/to/somewhere/' -e 'pattern'

# Search only files with a specific extension
grep --include=\*.{c,h} -rnw '/path/to/somewhere/' -e "pattern"

# Excluding files with a certain extension
grep --exclude=\*.o -rnw '/path/to/somewhere/' -e "pattern"

# Excluding specific directories
grep --exclude-dir={dir1,dir2,*.dst} -rnw '/path/to/somewhere/' -e "pattern"
```

## References

- [SO: Finding a specific string with grep](https://stackoverflow.com/questions/16956810/how-to-find-all-files-containing-specific-text-string-on-linux)
