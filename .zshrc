export LANG=ja_JP.UTF-8
setopt auto_cd
setopt auto_pushd
setopt correct
setopt list_packed
setopt nolistbeep

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

## Command history
#
HISTFILE=~/.zsh_history
HISTSIZE=10000
SAVEHIST=10000
setopt hist_ignore_dups
setopt share_history
autoload history-search-end
zle -N history-beginning-search-backward-end history-search-end
zle -N history-beginning-search-forward history-search-end
bindkey "^P" history-beginning-search-backward-end
bindkey "^N" history-beginning-search-forward-end

## Completion configuration
#
autoload -U compinit; compinit

export EDITOR=vim

## regex as PCRE-compatible
setopt re_match_pcre

## 
setopt print_eight_bit

setopt interactive_comments

setopt auto_menu

bindkey -e
bindkey "^[[Z" reverse-menu-complete  # Shift+Tabで補完候補を逆順に移動
#bindkey "^R" history-incremental-search-backward

setopt extended_glob

## Color configuration
autoload -Uz colors; colors
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
export ZLS_COLORS=$LS_COLORS
# colorize completion files same as `ls`
zstyle ':completion:*:default' list-colors ${(s.:.)LS_COLORS}

# command aliases
alias la='ls -la'

# for Git
#zstyle ':completion:*:*:git:*' script  ~/apps/zsh/git-completion.zsh
. ~/apps/zsh/git-prompt.sh
GIT_PS1_SHOWDIRTYSTATE=true
GIT_PS1_SHOWUNTRACKEDFILES=true
GIT_PS1_SHOWSTASHSTATE=true
GIT_PS1_SHOWUPSTREAM=auto

## Prompt configuration
#
setopt prompt_subst

PROMPT="%{${fg_bold[green]}%}%D{%m/%d(%a)} %* %{${fg[yellow]}%}[%m]:%~%{${reset_color}%}
[%n@ %c]$ "

PROMPT2='[%n]>'

# もしかして時のプロンプト指定
SPROMPT="%{$fg_bold[white]%}%{$suggest%}もしかして${reset_color} %{$fg_bold[red]%}%B%r%b %{$fg_bold[white]%}? [y,n,a,e]:${reset_color} "

RPROMPT='%B%F{cyan}$(__git_ps1 "[%s]")%b%F{white}'

setopt transient_rprompt

# added by Anaconda2 4.0.0 installer
export PATH="/home/sakurai/anaconda2/bin:$PATH"

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

#########################################################
# Aliases
#########################################################
alias finds='find 2>/dev/null'
alias fn='finds -name'
alias fnr='finds / -name'
alias diff='colordiff -u'


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
source /opt/ros/indigo/setup.zsh
source ~/catkin_ws/devel/setup.zsh
export ROS_HOSTNAME=sakurai-Trusty64
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
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'

#########################################################
# 日本語関連
#########################################################
# 半角/全角キーの点滅を抑制する
xset -r 49
