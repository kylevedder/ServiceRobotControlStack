#!/bin/bash
changed_files=$(git diff --cached --name-only);
modified_files=$(git ls-files -m);

for added_file in $changed_files; do
    for modified_file in $modified_files; do
        if [[ $added_file == $modified_file ]]; then
            if whiptail --yesno --defaultno "File $added_file is both added and modified. Are you REALLY sure you want to continue? Think really hard before answering yes." 20 60 ;then
                echo "Taking added version, not modified version."
            else
                echo "Quitting, as you selected to not commit both added and modified files."
                [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
            fi
        fi
    done
done

# Walk through each file
for file in $changed_files; do
    if [[ $file == *"control_stack/"* ]]; then
        file_contents=$(cat $file)
        if [ -z "$file_contents" ]
        then
            echo -e  "\e[33mSkipping empty file '$file'\e[0m"
            continue
        fi
        if [[ $file == *"Makefile" ]]
        then
            echo -e "\e[33mWarning: Modified Makefile\e[0m";
        fi

        # Restrict to only .cpp, .cc, .h, and .lua files
        if [[ $file == *".cpp" ]] || [[ $file == *".cc" ]] || [[ $file == *".h" ]] || [[ $file == *".lua" ]]
        then
            # Run Copyright checker against the file
            ci/check_copyright_block.py "$file";
            res=$?;
            if [ "$res" -ne 0 ];then
                echo -e "\033[1m\e[31mCopyright Check Failures!!! File: $file\e[0m";
                exit 1;
            fi
        fi

        # Run Google C++ Linter against the file.
        ci/cpplint.py "$file";
        res=$?;
        if [ "$res" -ne 0 ];then
            echo -e "\033[1m\e[31mCommit Lint Failures!!! File: $file\e[0m";
            exit 1;
        fi

#        if [[ $file == *".lua" ]]
#        then
#            # Run luacheck linter against the file
#            scripts/luacheck/bin/luacheck -q --config scripts/luacheck_config "$file";
#            res=$?;
#            if [ "$res" -ne 0 ];then
#                echo -e "\033[1m\e[31mCommit Lint Failures!!! File: $file\e[0m";
#                exit 1;
#            fi
#        fi
    fi
done


# Build to make sure the build isn't broken...
./build.sh
res=$?
if [ "$res" -ne 0 ];then
    echo -e "\033[1m\e[31mYou broke the build!!!\e[0m";
    exit 1;
fi

# Allows us to read user input below, assigns stdin to keyboard
exec < /dev/tty

# List all unknown files (Should help catch unadded files)
unknown=$(git ls-files --others --exclude-standard)
if [ -n "$unknown" ]
then
   echo -e "\e[33mWarning: Unadded files: >>>>>>>>>>>>>>\e[0m";
   echo "$unknown"
   echo -e "\e[33m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m";
   response=""
   while [[ -z "${response// }" ]]; do
       read -r -p "Are you sure you want to ignore these files? [y/n] " response
       response="${response//[$'\\\t\\\r\\\n']}"
       if [[ -n "${response// }" ]]; then
           case "$response" in
               [yY][eE][sS]|[yY]) 
                   echo "Continuing with commit"
                   ;;
               *)
                   echo "Aborting commit"
                   exit -1
                   ;;
           esac
       fi
   done
fi

echo -e "\033[1m\e[32mCommit Lint Passed...\e[0m";
exit 0;
