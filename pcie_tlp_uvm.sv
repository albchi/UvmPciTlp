//----------------------------------------------------------------------
// pcie_if.sv: Defines the virtual interface for the PCIe TLP
//----------------------------------------------------------------------
interface pcie_if(input bit clk);
  logic rst_n;
  logic valid;
  logic ready;
  logic [31:0] addr;
  logic [3:0] tlp_type; // Corresponds to tlp_type_e
  logic [31:0] wdata;
  logic [3:0] wstrb; // Write strobes for byte enables
  
  // Driver clocking block
  clocking driver_cb @(posedge clk);
    output valid, addr, tlp_type, wdata, wstrb;
    input ready;
  endclocking

  // Monitor clocking block
  clocking monitor_cb @(posedge clk);
    input valid, ready, addr, tlp_type, wdata, wstrb;
  endclocking

endinterface

//----------------------------------------------------------------------
// pcie_tlp_item.sv: Defines the PCIe TLP transaction item
//----------------------------------------------------------------------
class pcie_tlp_item extends uvm_sequence_item;

  // TLP Type enumeration
  typedef enum { MEM_RD, MEM_WR, CFG_RD, CFG_WR } tlp_type_e;

  //--- Randomized Properties ---//
  rand tlp_type_e tlp_type; // Type of TLP (Memory/Config, Read/Write)
  rand bit[31:0] address;    // 32-bit address
  rand int        length;     // Length of the data payload in DWORDS (32-bit words)
  rand byte       data[];     // Dynamic array for data payload

  // UVM Field Macros for automation (print, copy, compare, etc.)
  `uvm_object_utils_begin(pcie_tlp_item)
    `uvm_field_enum(tlp_type_e, tlp_type, UVM_ALL_ON)
    `uvm_field_int(address, UVM_ALL_ON | UVM_HEX)
    `uvm_field_int(length, UVM_ALL_ON)
    `uvm_field_array_int(data, UVM_ALL_ON | UVM_HEX)
  `uvm_object_utils_end

  // Constructor
  function new(string name = "pcie_tlp_item");
    super.new(name);
  endfunction

  //--- Constraints for valid TLP generation ---//
  constraint c_length { length inside {[1:64]}; }
  constraint c_addr_aligned { address[1:0] == 2'b00; }

  function void post_randomize();
    if (tlp_type == MEM_WR || tlp_type == CFG_WR) begin
      data = new[length * 4];
      foreach (data[i]) begin
        data[i] = $urandom();
      end
    end else begin
      data = new[0];
    end
  endfunction

endclass

//----------------------------------------------------------------------
// random_tlp_sequence.sv: Sequence to generate a stream of random TLPs
//----------------------------------------------------------------------
class random_tlp_sequence extends uvm_sequence #(pcie_tlp_item);

  `uvm_object_utils(random_tlp_sequence)

  int num_tlps = 10;

  function new(string name = "random_tlp_sequence");
    super.new(name);
  endfunction

  virtual task body();
    if (!uvm_config_db#(int)::get(m_sequencer, "", "num_tlps", num_tlps)) begin
       `uvm_warning("SEQ_CFG", "Could not get 'num_tlps' from config_db. Using default.")
    end
    
    `uvm_info("SEQ", $sformatf("Generating %0d TLPs", num_tlps), UVM_MEDIUM)

    repeat (num_tlps) begin
      pcie_tlp_item req = pcie_tlp_item::type_id::create("req");
      start_item(req);
      if (!req.randomize()) begin
        `uvm_error("SEQ", "Failed to randomize TLP request")
      end
      `uvm_info("SEQ", $sformatf("Generated TLP: %s", req.sprint()), UVM_MEDIUM)
      finish_item(req);
    end
  endtask

endclass

//----------------------------------------------------------------------
// pcie_driver.sv: Drives the TLP transactions onto the virtual interface
//----------------------------------------------------------------------
class pcie_driver extends uvm_driver #(pcie_tlp_item);

  `uvm_component_utils(pcie_driver)
  
  // Virtual interface handle
  virtual pcie_if vif;

  function new(string name = "pcie_driver", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    // Get the virtual interface from the config_db
    if (!uvm_config_db#(virtual pcie_if)::get(this, "", "pcie_vif", vif)) begin
      `uvm_fatal(get_type_name(), "Failed to get virtual interface handle from config_db")
    end
  endfunction

  // Main driver task to get and drive transactions
  virtual task run_phase(uvm_phase phase);
    forever begin
      pcie_tlp_item req;
      seq_item_port.get_next_item(req);
      `uvm_info(get_type_name(), $sformatf("Driving TLP:\n%s", req.sprint()), UVM_LOW)
      drive_tlp(req);
      seq_item_port.item_done();
    end
  endtask

  // Task to drive a single TLP onto the interface
  protected virtual task drive_tlp(pcie_tlp_item item);
    // Drive header information
    vif.driver_cb.valid <= 1;
    vif.driver_cb.tlp_type <= item.tlp_type;
    vif.driver_cb.addr <= item.address;
    
    // For writes, drive the data payload cycle by cycle
    if(item.tlp_type == pcie_tlp_item::MEM_WR || item.tlp_type == pcie_tlp_item::CFG_WR) begin
      for (int i = 0; i < item.length; i++) begin
        @(vif.driver_cb);
        // Wait for DUT to be ready
        wait (vif.driver_cb.ready === 1);
        vif.driver_cb.wdata <= {item.data[i*4+3], item.data[i*4+2], item.data[i*4+1], item.data[i*4]};
        vif.driver_cb.wstrb <= 4'b1111; // Assuming full DWORD write
      end
    end else begin // For reads, just one cycle for the header
       @(vif.driver_cb);
       wait (vif.driver_cb.ready === 1);
    end
    
    // De-assert valid after transaction is done
    @(vif.driver_cb);
    vif.driver_cb.valid <= 0;
  endtask

endclass

//----------------------------------------------------------------------
// pcie_agent.sv: Encapsulates the driver and sequencer
//----------------------------------------------------------------------
class pcie_agent extends uvm_agent;
  
  pcie_driver   m_driver;
  uvm_sequencer #(pcie_tlp_item) m_sequencer;

  `uvm_component_utils(pcie_agent)

  function new(string name = "pcie_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    m_sequencer = uvm_sequencer #(pcie_tlp_item)::type_id::create("m_sequencer", this);
    m_driver = pcie_driver::type_id::create("m_driver", this);
  endfunction

  virtual function void connect_phase(uvm_phase phase);
    m_driver.seq_item_port.connect(m_sequencer.seq_item_export);
  endfunction

endclass

//----------------------------------------------------------------------
// pcie_env.sv: The environment component
//----------------------------------------------------------------------
class pcie_env extends uvm_env;

  pcie_agent m_pcie_agent;

  `uvm_component_utils(pcie_env)

  function new(string name = "pcie_env", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    m_pcie_agent = pcie_agent::type_id::create("m_pcie_agent", this);
  endfunction

endclass

//----------------------------------------------------------------------
// base_test.sv: Test to run the random sequence
//----------------------------------------------------------------------
class base_test extends uvm_test;
  
  pcie_env m_pcie_env;

  `uvm_component_utils(base_test)

  function new(string name = "base_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    m_pcie_env = pcie_env::type_id::create("m_pcie_env", this);
    uvm_config_db#(int)::set(this, "m_pcie_env.m_pcie_agent.m_sequencer", "num_tlps", 20);
  endfunction

  virtual task run_phase(uvm_phase phase);
    random_tlp_sequence seq = random_tlp_sequence::type_id::create("seq");
    phase.raise_objection(this);
    `uvm_info("TEST", "Starting the random_tlp_sequence", UVM_MEDIUM)
    seq.start(m_pcie_env.m_pcie_agent.m_sequencer);
    phase.drop_objection(this);
  endtask

endclass


//----------------------------------------------------------------------
// top_module.sv: Top-level module to kick off the UVM test
//----------------------------------------------------------------------
module top_module;
  import uvm_pkg::*;
  
  bit clk;
  always #5 clk = ~clk;

  // Instantiate the interface
  pcie_if pif(clk);

  initial begin
    // Waveform dumping
    $dumpfile("dump.vcd");
    $dumpvars(0, top_module);

    // Make the virtual interface available to all components under uvm_test_top
    uvm_config_db#(virtual pcie_if)::set(null, "uvm_test_top.*", "pcie_vif", pif);
    
    // Run the 'base_test'
    run_test("base_test");
  end

  // Simple DUT model that makes 'ready' high
  initial begin
    pif.ready = 1;
  end

endmodule


