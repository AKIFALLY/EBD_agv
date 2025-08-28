#!/bin/bash

# Script to update remaining this.currentFlow references to use taflFlowStore

# Simple property accesses
sed -i 's/this\.currentFlow\.metadata\./taflFlowStore.getFlow().metadata./g' tafl-editor.js
sed -i 's/this\.currentFlow\.settings\./taflFlowStore.getFlow().settings./g' tafl-editor.js
sed -i 's/this\.currentFlow\.variables/taflFlowStore.getFlow().variables/g' tafl-editor.js
sed -i 's/this\.currentFlow\.preload/taflFlowStore.getFlow().preload/g' tafl-editor.js
sed -i 's/this\.currentFlow\.rules/taflFlowStore.getFlow().rules/g' tafl-editor.js
sed -i 's/this\.currentFlow\.flow/taflFlowStore.getFlow().flow/g' tafl-editor.js

# Generic this.currentFlow access
sed -i 's/this\.currentFlow/taflFlowStore.getFlow()/g' tafl-editor.js

# Fix this.isDirty
sed -i 's/this\.isDirty = true/taflFlowStore.setDirty(true)/g' tafl-editor.js
sed -i 's/this\.isDirty = false/taflFlowStore.setDirty(false)/g' tafl-editor.js
sed -i 's/this\.isDirty/taflFlowStore.isDirty()/g' tafl-editor.js

echo "Updated references to use taflFlowStore"