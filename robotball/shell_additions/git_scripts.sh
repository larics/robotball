GIT_MAIL="$HOSTNAME@fakemail.com"

function git {
  if [[ "$1" == "commit" && "$@" != *"--help"* ]]; then
    shift 1
    read -p "Please enter your name: " GIT_USER
    command git commit --author="$GIT_USER <$GIT_MAIL>" "$@"
  else
    command git "$@"
  fi
}
