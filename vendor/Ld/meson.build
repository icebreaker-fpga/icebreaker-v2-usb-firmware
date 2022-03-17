linker_script = files('Link.ld')
add_global_link_arguments([
  '-T@0@'.format(meson.project_source_root() / '@0@'.format(linker_script[0])),
], language: ['c', 'cpp'])