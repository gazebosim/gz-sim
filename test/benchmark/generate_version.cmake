execute_process(
  COMMAND hg id --id
  WORKING_DIRECTORY ${repository_root}
  OUTPUT_VARIABLE HG_GLOBAL_REVISION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND hg id --num
  WORKING_DIRECTORY ${repository_root}
  OUTPUT_VARIABLE HG_REVISION_NUM
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND hg id --branch
  WORKING_DIRECTORY ${repository_root}
  OUTPUT_VARIABLE HG_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(TIMESTAMP build_time)

configure_file(${input_file} ${output_file})

