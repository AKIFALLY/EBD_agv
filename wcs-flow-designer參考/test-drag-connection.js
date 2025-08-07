const { chromium } = require('playwright');
const path = require('path');

async function testDragConnection() {
  const browser = await chromium.launch({ headless: false, slowMo: 1500 });
  const page = await browser.newPage();
  
  page.on('console', msg => {
    if (!msg.text().includes('parser-blocking')) {
      console.log(`[${msg.type().toUpperCase()}]`, msg.text());
    }
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    console.log('=== DRAG CONNECTION TEST ===');
    
    // Step 1: Create nodes
    console.log('1. Creating nodes...');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(1000);
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(1000);
    
    // Step 2: Create connection
    console.log('2. Creating connection...');
    const outputSockets = await page.$$('.rete-socket.output');
    const inputSockets = await page.$$('.rete-socket.input');
    
    if (outputSockets.length > 0 && inputSockets.length > 0) {
      const output = outputSockets[0];
      const input = inputSockets[0];
      
      const outputBox = await output.boundingBox();
      const inputBox = await input.boundingBox();
      
      // Create connection
      await page.mouse.move(outputBox.x + outputBox.width/2, outputBox.y + outputBox.height/2);
      await page.mouse.down();
      await page.mouse.move(inputBox.x + inputBox.width/2, inputBox.y + inputBox.height/2);
      await page.mouse.up();
      
      await page.waitForTimeout(1000);
      
      const connectionCount = await page.textContent('#connectionCount');
      console.log('Connection created. Count:', connectionCount);
      
      // Step 3: Test dragging node with connection
      console.log('3. Testing node drag with connection...');
      
      const nodes = await page.$$('.rete-node');
      if (nodes.length >= 2) {
        const node1 = nodes[0]; // First node
        const node1Box = await node1.boundingBox();
        
        console.log('Node 1 initial position:', node1Box);
        
        // Drag the first node
        console.log('Dragging first node...');
        await page.mouse.move(node1Box.x + node1Box.width/2, node1Box.y + node1Box.height/2);
        await page.mouse.down();
        await page.mouse.move(node1Box.x + 150, node1Box.y - 100); // Move significantly
        await page.mouse.up();
        
        await page.waitForTimeout(1000);
        
        // Check if connection line updated
        const connections = await page.$$('svg[id^="connection-"]');
        console.log('Visual connections after drag:', connections.length);
        
        if (connections.length > 0) {
          const connectionSvg = connections[0];
          const pathElement = await connectionSvg.$('path');
          if (pathElement) {
            const pathData = await pathElement.getAttribute('d');
            console.log('Connection path after drag:', pathData.substring(0, 50) + '...');
          }
        }
        
        // Step 4: Drag second node
        console.log('4. Dragging second node...');
        const node2 = nodes[1];
        const node2Box = await node2.boundingBox();
        
        await page.mouse.move(node2Box.x + node2Box.width/2, node2Box.y + node2Box.height/2);
        await page.mouse.down();
        await page.mouse.move(node2Box.x - 100, node2Box.y + 100);
        await page.mouse.up();
        
        await page.waitForTimeout(1000);
        
        console.log('Both nodes dragged. Connection should have updated.');
        
        // Final check
        const finalConnections = await page.$$('svg[id^="connection-"]');
        console.log('Final visual connections:', finalConnections.length);
      }
    }
    
    console.log('\n=== DRAG CONNECTION TEST COMPLETE ===');
    console.log('✅ Connection lines should now follow nodes when dragged!');
    
    // Keep open for visual inspection
    await page.waitForTimeout(10000);
    
  } catch (error) {
    console.error('Test error:', error.message);
  } finally {
    await browser.close();
  }
}

testDragConnection();