#!/bin/bash
# Download CodeMirror mode files
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/mode/yaml/yaml.min.js -O codemirror-mode-yaml.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/mode/javascript/javascript.min.js -O codemirror-mode-javascript.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/mode/overlay.min.js -O codemirror-mode-overlay.min.js

# Download fold addons
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/fold/foldcode.min.js -O codemirror-fold-foldcode.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/fold/foldgutter.min.js -O codemirror-fold-foldgutter.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/fold/indent-fold.min.js -O codemirror-fold-indent.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/fold/brace-fold.min.js -O codemirror-fold-brace.min.js

# Download edit addons
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/edit/matchbrackets.min.js -O codemirror-edit-matchbrackets.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/edit/closebrackets.min.js -O codemirror-edit-closebrackets.min.js

# Download lint addons
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/lint/lint.min.js -O codemirror-lint.min.js
wget -q https://cdnjs.cloudflare.com/ajax/libs/codemirror/5.65.2/addon/lint/json-lint.min.js -O codemirror-lint-json.min.js

echo "All CodeMirror JS files downloaded"
