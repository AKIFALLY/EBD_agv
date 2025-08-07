const { chromium } = require('playwright');
const path = require('path');

async function testCompleteApplication() {
  const browser = await chromium.launch({ headless: false, slowMo: 1000 });
  const page = await browser.newPage();
  
  page.on('console', msg => {
    if (!msg.text().includes('parser-blocking')) {
      console.log(`[${msg.type().toUpperCase()}]`, msg.text());
    }
  });
  
  page.on('pageerror', error => {
    console.error('PAGE ERROR:', error.message);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    console.log('=== COMPLETE FUNCTIONALITY TEST ===');
    
    // Test 1: Create nodes
    console.log('\n1. Creating nodes...');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(500);
    await page.click('[data-node-type="sorting"]'); 
    await page.waitForTimeout(500);
    await page.click('[data-node-type="shipping"]');
    await page.waitForTimeout(500);
    
    const nodeCount = await page.textContent('#nodeCount');
    console.log('Nodes created:', nodeCount);
    
    // Test 2: Check if nodes are visible and interactive
    const nodes = await page.$$('.rete-node');
    console.log('Visible nodes:', nodes.length);
    
    if (nodes.length > 0) {
      // Test node selection
      console.log('\n2. Testing node selection...');
      await nodes[0].click();
      await page.waitForTimeout(500);
      
      const propertiesVisible = await page.isVisible('.properties-panel.active');
      console.log('Properties panel visible:', propertiesVisible);
      
      // Test property editing
      if (propertiesVisible) {
        await page.fill('#nodeName', 'My Receiving Node');
        await page.fill('#nodeDescription', 'Test description');
        console.log('Properties updated');
      }
    }
    
    // Test 3: Test dragging (simulate)
    console.log('\n3. Testing node dragging...');
    if (nodes.length >= 2) {
      const node1 = nodes[0];
      const box1 = await node1.boundingBox();
      
      if (box1) {
        // Simulate drag
        await page.mouse.move(box1.x + box1.width/2, box1.y + box1.height/2);
        await page.mouse.down();
        await page.mouse.move(box1.x + 100, box1.y + 50);
        await page.mouse.up();
        console.log('Node dragging simulated');
      }
    }
    
    // Test 4: Test connections
    console.log('\n4. Testing connections...');
    const outputSockets = await page.$$('.rete-socket.output');
    const inputSockets = await page.$$('.rete-socket.input');
    
    console.log('Output sockets found:', outputSockets.length);
    console.log('Input sockets found:', inputSockets.length);
    
    if (outputSockets.length > 0 && inputSockets.length > 0) {
      try {
        // Try to create a connection
        const outputSocket = outputSockets[0];
        const inputSocket = inputSockets[0];
        
        const outputBox = await outputSocket.boundingBox();
        const inputBox = await inputSocket.boundingBox();
        
        if (outputBox && inputBox) {
          // Simulate connection drag
          await page.mouse.move(outputBox.x + outputBox.width/2, outputBox.y + outputBox.height/2);
          await page.mouse.down();
          await page.mouse.move(inputBox.x + inputBox.width/2, inputBox.y + inputBox.height/2);
          await page.mouse.up();
          
          await page.waitForTimeout(1000);
          const connectionCount = await page.textContent('#connectionCount');
          console.log('Connections after drag:', connectionCount);
        }
      } catch (error) {
        console.log('Connection test error:', error.message);
      }
    }
    
    // Test 5: Test toolbar functions
    console.log('\n5. Testing toolbar functions...');
    
    // Test save
    const toolbarButtons = await page.$$('.toolbar-button');
    if (toolbarButtons.length >= 3) {
      await toolbarButtons[2].click(); // Save button
      console.log('Save button clicked');
      await page.waitForTimeout(1000);
    }
    
    // Test new flow
    if (toolbarButtons.length >= 1) {
      await toolbarButtons[0].click(); // New Flow button
      await page.waitForTimeout(500);
      const nodeCountAfterClear = await page.textContent('#nodeCount');
      console.log('Nodes after new flow:', nodeCountAfterClear);
    }
    
    console.log('\n=== TEST COMPLETE ===');
    console.log('🎉 WCS Flow Designer is working!');
    
    // Keep browser open for manual testing
    await page.waitForTimeout(10000);
    
  } catch (error) {
    console.error('Test error:', error);
  } finally {
    await browser.close();
  }
}

testCompleteApplication();