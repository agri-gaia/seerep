# Style file for markdownlint

all

exclude_rule 'fenced-code-language' # Fenced code blocks should have a language specified
exclude_rule 'no-inline-html' # This rule is triggered whenever raw HTML is used in a markdown document:
exclude_rule 'code-block-style'

# Line lenght
rule 'MD013', :line_length => 120, :ignore_code_blocks => true, :tables => false

# Unordered list indentation
rule 'MD007', :indent => 4

# Ordered list item prefix
rule 'MD029', :style => 'ordered'
