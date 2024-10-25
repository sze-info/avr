---
layout: default
title: Linux
parent: Self-paced Tasks
---

# Linux Practice

## Creating Directory Structure - Terminal

To complete this task, install the `tree` command:
```
sudo apt install tree
```

You will need the `touch`, `chmod`, and `mkdir` commands to complete this task.

Create the following directory structure, which should look like this when you run `cd ~ && tree tmp_dir/`:

```
~/tmp_dir/
├── animals
│   ├── cat
│   └── dog
│       ├── komondor
│       ├── puli
│       └── vizsla
├── colors
│   ├── blue
│   ├── green
│   └── red
├── py_exec.py
├── simple_text.txt
└── top
    └── middle
        └── bottom
            └── hello.txt
13 directories, 3 files
```

The `ls -l  ~/tmp_dir/` command should show `rwx` values similar to the following:

```
drwxr-xr-x 4 he he 4096 Feb 17 14:54 animals
drwxr-xr-x 5 he he 4096 Feb 17 14:55 colors
-rwxrwxrwx 1 he he    0 Feb 17 14:52 py_exec.py
-rw-r--r-- 1 he he    0 Feb 17 14:53 simple_text.txt
drwxr-xr-x 3 he he 4096 Feb 17 14:43 top
```

### Solution Guide

```
mkdir -p top/middle/bottom
mkdir -p colors/{red,green,blue}
mkdir -p animals/{cat,dog/{vizsla,puli,komondor}}
```

## Text Files

If not already created, create a `~/tmp_text/` directory.

Inside the directory, create a `hello.py` file and populate it with the following content from the terminal:

``` python
import sys
print('\nHello vilag!\nA verzio pedig:\n' + sys.version)
```
Make it executable and run it.

### Solution Guide

```bash
chmod +x hello.py
./hello.py
```