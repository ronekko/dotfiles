"dein Scripts-----------------------------
if &compatible
  set nocompatible               " Be iMproved
endif

" Required:
let s:dein_dir = expand('~/.vim/dein')
let s:dein_repo_dir = s:dein_dir . '/repos/github.com/Shougo/dein.vim'

" もしdeinがインストールされていなければcloneしてくる
if &runtimepath !~# '/dein.vim'
  if !isdirectory(s:dein_repo_dir)
    execute '!git clone https://github.com/Shougo/dein.vim ' s:dein_repo_dir
  endif
  execute 'set runtimepath^=' . s:dein_repo_dir
endif
execute 'set runtimepath^=' . s:dein_repo_dir

" Required:
if dein#load_state(s:dein_dir)
  call dein#begin(s:dein_dir)

  " Let dein manage dein
  " Required:
  call dein#add('Shougo/dein.vim')

  " Add or remove your plugins here:
  call dein#add('Shougo/neosnippet.vim')
  call dein#add('Shougo/neosnippet-snippets')
  call dein#add('tyru/caw.vim.git') " コメントのトグル
  call dein#add('itchyny/lightline.vim')
  call dein#add('ctrlpvim/ctrlp.vim')

  " You can specify revision/branch/tag.
  call dein#add('Shougo/vimshell', { 'rev': '3787e5' })

  " Required:
  call dein#end()
  call dein#save_state()
endif

" Required:
filetype plugin indent on

" If you want to install not installed plugins on startup.
if dein#check_install()
  call dein#install()
endif

"End dein Scripts-------------------------

" 行番号を表示する
set number

" 編集中のファイル名を表示する
set title

" ステータスライン関連の設定
set laststatus=2

" 前回終了したカーソル行に移動する
autocmd BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$") | exe "normal g`\"" | endif

" タブ幅の設定
set expandtab
set tabstop=2
set softtabstop=2
set shiftwidth=2
set autoindent
set smartindent

" 検索の挙動に関する設定:
set ignorecase " 検索時に大文字小文字を虫（noignorecase:無視しない）
set smartcase  " 大文字小文字の両方が含まれている場合は大文字小文字を区別する
set incsearch  " インクリメンタルサーチ
set hlsearch   " 検索ヒット箇所のハイライト

" 色関連の設定
set t_Co=256
"colorscheme peachpuff
colorscheme molokai

" not allow to change texts in the terminal's title bar
set notitle

"-------------------------
" Key bind
"-------------------------
" Ctrl+\ toggles comment/uncomment
nmap <C-\> <Plug>(caw:hatpos:toggle)
vmap <C-\> <Plug>(caw:hatpos:toggle)

"-------------------------
" lightline
"-------------------------
if !has('gui_running')
  set t_Co=256
endif
" let g:lightline = {'colorscheme': 'wombat'}
" let g:lightline = {'colorscheme': 'solarized'}

"-------------------------
" CtrlP
"-------------------------
let g:ctrlp_show_hidden         = 1   " dotfileを検索対象にする
let g:ctrlp_clear_cache_on_exit = 0   " 終了時キャッシュをクリアしない
let g:ctrlp_mruf_max            = 500 " MRUの最大記録数
let g:ctrlp_open_new_file       = 1   " 新規ファイル作成時にタブで開く

"-------------------------
"  XMLの閉じタグを自動的に補完する
"-------------------------
augroup XmlAutoCloser
  autocmd!
  autocmd Filetype xml inoremap <buffer> </ </<C-x><C-o>
  autocmd Filetype html inoremap <buffer> </ </<C-x><C-o>
augroup END

