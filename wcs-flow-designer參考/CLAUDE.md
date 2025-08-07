# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Application Overview

This is a **WCS (Warehouse Control System) Flow Designer** - a single-page web application built with Rete.js v2 and pure JavaScript. The application provides a visual node-based editor for designing warehouse material flow processes and control systems.

## Architecture

### Single File Application
- **index.html**: Contains the complete application - HTML structure, CSS styling, and JavaScript logic in one file
- **Self-contained**: No build process required, runs directly in browser
- **CDN Dependencies**: Rete.js v2 libraries loaded via jsDelivr CDN

### Core Components
- **WcsFlowDesigner Class**: Main application controller that manages the Rete.js editor instance
- **Node Registration System**: Defines 14 WCS-specific node types across 5 categories
- **Event System**: Handles UI interactions, node selection, and property editing

### Node Architecture
The application implements a categorized node system for warehouse operations:

- **Input Nodes**: `receiving`, `scanner`, `goods-in` (green)
- **Process Nodes**: `sorting`, `quality-check`, `packaging` (blue) 
- **Control Nodes**: `plc-controller`, `decision`, `timer` (orange)
- **Storage Nodes**: `rack`, `buffer` (purple)
- **Output Nodes**: `shipping`, `agv-dispatch`, `printer` (red)

Each node type has configurable properties: name, description, capacity, and processing time.

### UI Layout
- **Sidebar**: Node palette organized by category
- **Main Editor**: Rete.js canvas for visual flow design
- **Properties Panel**: Overlay panel for selected node configuration
- **Toolbar**: Flow management controls (New, Load, Save, Export)
- **Status Bar**: Real-time statistics display

## Running the Application

Open `index.html` directly in any modern web browser. No build process, server, or installation required.

## Key Technical Details

### Rete.js Integration
- **Version**: 2.0.6 with Area and Connection plugins
- **Editor ID**: `'wcs-flow@0.1.0'`
- **Canvas Settings**: 5000x4000 virtual space, 0.1-2x zoom range
- **Socket System**: Single 'flow' socket type for all connections

### Node Connection Logic
- Input nodes: Output socket only
- Output nodes: Input socket only  
- Process/Control/Storage nodes: Both input and output sockets
- Decision nodes: Additional 'Yes'/'No' output sockets

### Data Persistence
- **Save**: Exports flow as JSON file download
- **Load**: Imports JSON file via file picker
- **Format**: Standard Rete.js JSON serialization

### Styling System
- **Theme**: Dark UI with professional warehouse control aesthetic
- **Node Colors**: Category-based color coding matching sidebar icons
- **Responsive**: Fixed 100vh layout with flexible sidebar

## Adding New Node Types

To add new node types, modify the `nodeTypes` object in `registerNodes()` method around line 419. Each node type requires:
- `name`: Display name
- `category`: One of 'input', 'process', 'control', 'storage', 'output'  
- `color`: Hex color for visual identification

The system automatically handles socket creation based on category.