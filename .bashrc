# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
  PS1='${debian_chroot:+($debian_chroot)}\n\[\033[1;32m\]\D{%m/%d(%a)} \t\[\033[00m\] \[\033[1;33m\][\H]:\w\[\033[00m\]\n[\u@ \W]\[\033[36m\]\$(__git_ps1)\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias finds='find 2>/dev/null'
alias fn='finds -name'
alias fnr='finds / -name'


# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
#[ -r /home/sakurai/.config/byobu/prompt ] && . /home/sakurai/.config/byobu/prompt   #byobu-prompt#

# added by Anaconda2 4.0.0 installer
export PATH="/home/sakurai/anaconda2/bin:$PATH"

# git-completion.bash / git-prompt.sh
#
if [ -f ~/apps/bash/.git-completion.bash ]; then
  source ~/apps/bash/.git-completion.bash
fi

if [ -f ~/apps/bash/.git-prompt.sh ]; then
  source ~/apps/bash/.git-prompt.sh
fi
GIT_PS1_SHOWDIRTYSTATE=true
GIT_PS1_SHOWUNTRACKEDFILES=true
GIT_PS1_SHOWSTASHSTATE=true
GIT_PS1_SHOWUPSTREAM=auto
export PS1="\n\[\033[1;32m\]\D{%m/%d(%a)} \t\[\033[00m\] \[\033[1;33m\][\H]:\w\[\033[00m\]\n[\u@ \W]\[\033[36m\]\$(__git_ps1)\[\033[00m\]\$ "



##########################################################
## Cuda
##########################################################
#export PATH=/usr/local/cuda-6.5/bin:$PATH
#export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib64:$LD_LIBRARY_PATH
#export LIBRARY_PATH=/usr/local/cuda-6.5/lib64:$LIBRARY_PATH

##########################################################
## export PYTHONPATH=$PYTHONPATH:~/anaconda/lib/python2.7
##########################################################
#export PYTHONPATH=$PYTHONPATH:~/caffe/python

#########################################################
# for byobu
#########################################################
export VTE_CJK_WIDTH=1

#########################################################
# pygmentized cat and less
# http://xyk.hatenablog.com/entry/2014/12/24/161507
#########################################################
# c and cl (syntax-highlighted alternative of cat)
alias c='pygmentize -O style=monokai -f terminal256 -g -O encoding=utf-8'
function cl() {
  c $1 | nl -n ln -b a
}
alias cl=cl

# less
export PATH=~/apps:$PATH
export LESS='-R'
export LESSOPEN='|lessfilter %s'

# colordiff
alias diff='colordiff'

#########################################################
# add my data directory to environment variable
#########################################################
export DATA_PATH=~/data

#########################################################
# rednose
#########################################################
export NOSE_REDNOSE=1

##########################################################
## v-rep
##########################################################
export PATH=$PATH:~/V-REP_PRO_EDU_V3_3_0_64_Linux

#########################################################
# for ROS
#########################################################
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=`hostname -I`
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'
alias cm='cd ~/catkin_ws && catkin_make'

#########################################################
# for RU-Glue
#########################################################
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

#########################################################
# for ALE
#########################################################
export PATH=~/apps/Arcade-Learning-Environment-0.5.1:$PATH
export ALE_DIR=~/apps/Arcade-Learning-Environment-0.5.1

#########################################################
# for commandline userbility  
#########################################################
export WORDCHARS='*?_-.[]~=&;!#$%^(){}<>'
