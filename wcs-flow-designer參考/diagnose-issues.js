const { chromium } = require('playwright');
const path = require('path');

async function diagnoseIssues() {
  const browser = await chromium.launch({ headless: false, slowMo: 500 });
  const page = await browser.newPage();
  
  // Capture all events
  page.on('console', msg => {
    console.log(`[${msg.type().toUpperCase()}]`, msg.text());
  });
  
  page.on('pageerror', error => {
    console.error('PAGE ERROR:', error.message);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    console.log('=== DIAGNOSING NODE AND CONNECTION ISSUES ===');
    
    // Test 1: Check if nodes are actually being added to the editor
    console.log('\n1. Testing node addition...');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(1000);
    
    const editorState = await page.evaluate(() => {
      const designer = window.wcsDesigner;
      if (!designer) return { error: 'Designer not found' };
      
      return {
        editorExists: !!designer.editor,
        areaExists: !!designer.area,
        nodesInEditor: designer.editor.getNodes().length,
        editorContainer: !!document.getElementById('rete'),
        socketExists: !!designer.socket,
        nodeTypesExists: !!designer.nodeTypes
      };
    });
    
    console.log('Editor state:', editorState);
    
    // Test 2: Check DOM for actual node elements
    console.log('\n2. Checking DOM for nodes...');
    const domNodes = await page.$$('.rete-node');
    console.log('DOM nodes found:', domNodes.length);
    
    // Test 3: Check if area plugin is working
    console.log('\n3. Testing area plugin...');
    const areaInfo = await page.evaluate(() => {
      const container = document.getElementById('rete');
      return {
        containerExists: !!container,
        containerHasChildren: container ? container.children.length : 0,
        containerStyles: container ? getComputedStyle(container).position : 'N/A'
      };
    });
    console.log('Area info:', areaInfo);
    
    // Test 4: Try to add another node and see what happens
    console.log('\n4. Adding another node...');
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(1000);
    
    const afterSecondNode = await page.evaluate(() => {
      return {
        nodesInEditor: window.wcsDesigner?.editor?.getNodes().length || 0,
        domNodes: document.querySelectorAll('.rete-node').length
      };
    });
    console.log('After second node:', afterSecondNode);
    
    // Test 5: Check connection functionality
    console.log('\n5. Testing connection plugin...');
    const connectionInfo = await page.evaluate(() => {
      return {
        connectionPluginLoaded: !!window.ReteConnectionPlugin,
        connectionPluginKeys: window.ReteConnectionPlugin ? Object.keys(window.ReteConnectionPlugin) : []
      };
    });
    console.log('Connection info:', connectionInfo);
    
    // Test 6: Try manual drag operation
    console.log('\n6. Testing manual interaction...');
    const reteContainer = await page.$('#rete');
    if (reteContainer) {
      const box = await reteContainer.boundingBox();
      console.log('Rete container box:', box);
      
      // Try clicking in the center of the container
      if (box) {
        await page.click('#rete', { position: { x: box.width / 2, y: box.height / 2 } });
        console.log('Clicked center of rete container');
      }
    }
    
    console.log('\n=== DIAGNOSIS COMPLETE ===');
    
    // Keep open for manual testing
    await page.waitForTimeout(10000);
    
  } catch (error) {
    console.error('Diagnosis error:', error);
  } finally {
    await browser.close();
  }
}

diagnoseIssues();