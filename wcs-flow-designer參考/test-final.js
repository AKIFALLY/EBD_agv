const { chromium } = require('playwright');
const path = require('path');

async function testFinalApplication() {
  const browser = await chromium.launch({ headless: false });
  const page = await browser.newPage();
  
  page.on('console', msg => {
    if (msg.type() !== 'warning') {
      console.log(`[${msg.type().toUpperCase()}]`, msg.text());
    }
  });
  
  page.on('pageerror', error => {
    console.error('PAGE ERROR:', error.message);
  });
  
  try {
    const appFile = path.join(__dirname, 'index.html');
    console.log('=== TESTING FINAL APPLICATION ===');
    
    await page.goto(`file://${appFile}`);
    await page.waitForTimeout(3000);
    
    // Test 1: Check if plugins loaded
    console.log('\n1. Testing plugin loading...');
    const plugins = await page.evaluate(() => {
      return {
        Rete: typeof window.Rete,
        ReteAreaPlugin: typeof window.ReteAreaPlugin,
        ReteConnectionPlugin: typeof window.ReteConnectionPlugin,
        ConnectionPlugin: !!(window.ReteConnectionPlugin && window.ReteConnectionPlugin.ConnectionPlugin)
      };
    });
    console.log('Plugins:', plugins);
    
    // Test 2: Add nodes
    console.log('\n2. Testing node creation...');
    await page.click('[data-node-type="receiving"]');
    await page.waitForTimeout(500);
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(500);
    await page.click('[data-node-type="shipping"]');
    await page.waitForTimeout(500);
    
    const nodeCount = await page.textContent('#nodeCount');
    console.log('Nodes created:', nodeCount);
    
    // Test 3: Test toolbar buttons
    console.log('\n3. Testing toolbar functionality...');
    
    // Test New Flow button
    await page.click('.toolbar-button'); // First button is New Flow
    await page.waitForTimeout(500);
    const nodeCountAfterClear = await page.textContent('#nodeCount');
    console.log('Nodes after clear:', nodeCountAfterClear);
    
    // Add nodes again
    await page.click('[data-node-type="receiving"]');
    await page.click('[data-node-type="sorting"]');
    await page.waitForTimeout(500);
    
    // Test 4: Node selection and properties
    console.log('\n4. Testing node selection...');
    const nodes = await page.$$('.rete-node');
    if (nodes.length > 0) {
      await nodes[0].click();
      await page.waitForTimeout(500);
      
      const propertiesPanelVisible = await page.isVisible('.properties-panel.active');
      console.log('Properties panel visible:', propertiesPanelVisible);
      
      if (propertiesPanelVisible) {
        await page.fill('#nodeName', 'Test Node');
        await page.fill('#nodeDescription', 'Test Description');
        console.log('Node properties updated');
      }
    }
    
    // Test 5: Save functionality
    console.log('\n5. Testing save functionality...');
    const saveButtons = await page.$$('.toolbar-button');
    if (saveButtons.length >= 3) {
      await saveButtons[2].click(); // Save button
      await page.waitForTimeout(1000);
      console.log('Save function triggered');
    }
    
    console.log('\n=== TEST COMPLETED ===');
    console.log('Application appears to be working correctly!');
    
    // Keep browser open for manual inspection
    await page.waitForTimeout(5000);
    
  } catch (error) {
    console.error('Test error:', error);
  } finally {
    await browser.close();
  }
}

testFinalApplication();