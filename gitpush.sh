MSG=$1

if [ -z "$MSG" ]; then
	MSG="commit_$( date + '%s' )"
fi

git add .
git commit -m ${MSG}
git push origin master

