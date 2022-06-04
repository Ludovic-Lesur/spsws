#!/bin/bash

sscanf() {
	local str="$1"
	local format="$2"
	[[ "$str" =~ $format ]]
}

# Touch files to force makefile operation.
touch ../src/main.c
touch ../inc/version.h

# Get current date.
date=`date -R`

# Execute git commands.
git_version=`git describe --long --always`
diff=`git diff ':(exclude)../inc/mode.h' ':(exclude)../.settings' ':(exclude)../script'`

# Extract fields
sscanf $git_version "SW(.+).(.+)-(.+)-g(.+)"

# Manage dirty flag.
dirty_flag=0
if [[ $diff ]]; then
	dirty_flag=1
fi

echo "/*" > ../inc/version.h
echo " * version.h" >> ../inc/version.h
echo " *" >> ../inc/version.h
echo " * Auto-generated on: $date" >> ../inc/version.h
echo " * Author: Ludo" >> ../inc/version.h
echo " */" >> ../inc/version.h
echo "" >> ../inc/version.h
echo "#ifndef __VERSION_H__" >> ../inc/version.h
echo "#define __VERSION_H__" >> ../inc/version.h
echo "" >> ../inc/version.h
echo "#define GIT_VERSION       \"$git_version\"" >> ../inc/version.h
echo "#define GIT_MAJOR_VERSION ${BASH_REMATCH[1]}" >> ../inc/version.h
echo "#define GIT_MINOR_VERSION ${BASH_REMATCH[2]}" >> ../inc/version.h
echo "#define GIT_COMMIT_INDEX  ${BASH_REMATCH[3]}" >> ../inc/version.h
echo "#define GIT_COMMIT_ID     0x${BASH_REMATCH[4]}" >> ../inc/version.h
echo "#define GIT_DIRTY_FLAG    $dirty_flag" >> ../inc/version.h
echo "" >> ../inc/version.h
echo "#endif /* __VERSION_H__ */" >> ../inc/version.h
