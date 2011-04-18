#! /bin/bash

# check wether the script is called as root or not
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 
fi

DEP_FILE=

# parse the arguments
while getopts “d:” OPTION
do
  case $OPTION in
    d)
       DEP_FILE=$OPTARG
       ;;
    ?)
       echo invalid argument $OPTION
       exit
       ;;
  esac
done

# check wether a dependencies file has been provided
if [ "$DEP_FILE" ]
then
  echo "using $DEP_FILE file for dependencies ..."
else
  echo "No dependencies file provided, aborting ..."  
  exit 
fi

LOCAL_REPO_PATH=/usr/local/src/iri

#check wether the repository directory exists or not
if [ ! -d $LOCAL_REPO_PATH ]
then
  mkdir $LOCAL_REPO_PATH
fi

while read curline; do
  COMPILE=0
  MAKE_DEP=0
  cd $LOCAL_REPO_PATH
  echo "checking dependency $curline ..."
  DEP_NAME=$(basename $curline)
  if [ -d "$DEP_NAME" ]
  then
    echo "The $DEP_NAME library is present ... checking updates."
    cd $DEP_NAME
    LOCAL_REV_NUM=$(svn info | sed -n 's/^Revision: \([0-9]\+\)$/\1/p')
    echo "Local $DEP_NAME library revision: $LOCAL_REV_NUM"
    REPO_REV_NUM=$(svn info | sed -n 's/^Last Changed Rev: \([0-9]\+\)$/\1/p')
    echo "Repository $DEP_NAME library revision: $REPO_REV_NUM"
    if [ $LOCAL_REV_NUM -gt $REPO_REV_NUM ] 
    then
      echo "There exist local changes that are not commited to the repository"
    else
      if [ $LOCAL_REV_NUM -lt $REPO_REV_NUM ] 
      then
        echo "Updating to the latest revision ..."
        svn update
        COMPILE=1
      else
        echo "Already at the latest revision."
        MAKE_DEP=1
      fi
    fi
  else
    echo "The $DEP_NAME library is not present ... downloading."
    svn checkout $curline
    cd $DEP_NAME
    COMPILE=1
  fi  
  cd build
  if [ $MAKE_DEP -eq 1 ] 
  then
    make dep
  fi
  # build if necessary
  if [ $COMPILE -eq 1 ] 
  then
    cmake ..
    make dep
    make
    make install
  fi
  #back to the root repository source path
  cd ../../
done < $DEP_FILE

