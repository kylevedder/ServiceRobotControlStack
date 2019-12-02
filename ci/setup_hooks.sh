#!/bin/bash
ln -s ../../ci/pre-commit.sh ./.git/hooks/pre-commit
ln -s ../../ci/pre-push.sh ./.git/hooks/pre-push
echo "What is your email that you include in the copyright header?"
read email
echo "$email" > .git/copyrightemail
