#!/bin/bash

sscanf() {
    local str="$1"
    local format="$2"
    [[ "$str" =~ $format ]]
}

# Touch files to force makefile operation.
touch ../application/src/main.c
touch ../application/inc/version.h

# Get current date.
date=`date -R`

# Execute git commands.
git_version=`git describe --long --always`
diff=`git diff ':(exclude)../application/inc/mode.h' ':(exclude)../.settings' ':(exclude)../script'`

# Extract fields
sscanf $git_version "sw([0-9]+).([0-9]+)-([0-9]+)-g(.+)"

# Manage dirty flag.
dirty_flag=0
if [[ $diff ]]; then
    dirty_flag=1
fi

echo "/*" > ../application/inc/version.h
echo " * version.h" >> ../application/inc/version.h
echo " *" >> ../application/inc/version.h
echo " * Auto-generated on: $date" >> ../application/inc/version.h
echo " * Author: Ludo" >> ../application/inc/version.h
echo " */" >> ../application/inc/version.h
echo "" >> ../application/inc/version.h
echo "#ifndef __VERSION_H__" >> ../application/inc/version.h
echo "#define __VERSION_H__" >> ../application/inc/version.h
echo "" >> ../application/inc/version.h
echo "#define GIT_VERSION       \"$git_version\"" >> ../application/inc/version.h
echo "#define GIT_MAJOR_VERSION ${BASH_REMATCH[1]}" >> ../application/inc/version.h
echo "#define GIT_MINOR_VERSION ${BASH_REMATCH[2]}" >> ../application/inc/version.h
echo "#define GIT_COMMIT_INDEX  ${BASH_REMATCH[3]}" >> ../application/inc/version.h
echo "#define GIT_COMMIT_ID     0x${BASH_REMATCH[4]}" >> ../application/inc/version.h
echo "#define GIT_DIRTY_FLAG    $dirty_flag" >> ../application/inc/version.h
echo "" >> ../application/inc/version.h
echo "#endif /* __VERSION_H__ */" >> ../application/inc/version.h
