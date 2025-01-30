#
# This is a bash shell fragment, intended to be sourced by scripts that
# want to work with git in the emc2 repo.
#
# Sets GIT_BRANCH to the passed in branch name; if none is passed in it
# attempts to detect the current branch (this will fail if the repo is in a
# detached HEAD state).
#
# Sets DEB_COMPONENT based on the branch.  Official release branches get
# their own component, all other branches go in "scratch".
#
# Sets GIT_TAG to the most recent signed tag (this will fall back to the
# most recent tag of any kind if no signed tag is found).
#
# shellcheck shell=bash


function githelper() {
    if [ -z "$1" ]; then
        GIT_BRANCH=$(git branch | grep -E '^\*' | cut -d ' ' -f 2)
        if [ "$GIT_BRANCH" = "(no" ]; then
            echo "'git branch' says we're not on a branch, pass one in as an argument" > /dev/null 1>&2
            return
        fi
    else
        GIT_BRANCH="$1"
    fi

    case $GIT_BRANCH in
        master)
            GIT_TAG_GLOB="v2.10.*"
            DEB_COMPONENT="master"
            ;;
        # release branches have names matching "number.number", which is awkward to express as a glob
        [0-9].[0-9] | [0-9].[0-9][0-9] | [0-9].[0-9][0-9][0-9] | [0-9][0-9].[0-9] | [0-9][0-9].[0-9][0-9] | [0-9][0-9].[0-9][0-9][0-9])
            GIT_TAG_GLOB="v${GIT_BRANCH}.*"
            DEB_COMPONENT=$GIT_BRANCH
            ;;
        v2.5_branch)
            GIT_TAG_GLOB="v2.5*"
            DEB_COMPONENT="v2.5_branch"
            ;;
        v2.4_branch)
            GIT_TAG_GLOB="v2.4*"
            DEB_COMPONENT="v2.4_branch"
            ;;
        *)
            GIT_TAG_GLOB="*"
            # Disable unused variable warnings on DEB_COMPONENT
            # shellcheck disable=SC2034
            DEB_COMPONENT="scratch"
            ;;
    esac


    # use the gnupg keyring from our git repo to verify signatures on the release tags
    GNUPGHOME=$(git rev-parse --show-toplevel)/gnupg
    export GNUPGHOME

    NEWEST_SIGNED_TAG_UTIME=-1
    NEWEST_UNSIGNED_TAG_UTIME=-1
    for TAG in $(git tag -l "$GIT_TAG_GLOB"); do
        if ! git cat-file tag "$TAG" > /dev/null 2> /dev/null; then
            continue
        fi

        TAG_UTIME=$(git cat-file tag "$TAG" | grep tagger | awk '{print $(NF-1)-$NF*36}')

        if git tag -v "$TAG" > /dev/null 2> /dev/null; then
            # it's a valid signed tag
            if [ "$TAG_UTIME" -gt "$NEWEST_SIGNED_TAG_UTIME" ]; then
                NEWEST_SIGNED_TAG=$TAG
                NEWEST_SIGNED_TAG_UTIME=$TAG_UTIME
            fi
        else
            # unsigned tag
            if [ "$TAG_UTIME" -gt "$NEWEST_UNSIGNED_TAG_UTIME" ]; then
                NEWEST_UNSIGNED_TAG=$TAG
                NEWEST_UNSIGNED_TAG_UTIME=$TAG_UTIME
            fi
        fi

    done

    if [ "$NEWEST_SIGNED_TAG_UTIME" -gt -1 ]; then
        # Disable unused variable warnings on GIT_TAG
        # shellcheck disable=SC2034
        GIT_TAG="$NEWEST_SIGNED_TAG"
        return
    fi

    if [ "$NEWEST_UNSIGNED_TAG_UTIME" -gt -1 ]; then
        echo "no signed tags found, falling back to unsigned tags" > /dev/null 1>&2
        # Disable unused variable warnings on GIT_TAG
        # shellcheck disable=SC2034
        GIT_TAG="$NEWEST_UNSIGNED_TAG"
        return
    fi

    echo "no annotated tags found, not even unsigned" > /dev/null 1>&2
}

