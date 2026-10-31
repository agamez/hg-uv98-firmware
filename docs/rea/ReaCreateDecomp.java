import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.ProgramContext;
import java.math.BigInteger;
import ghidra.program.model.lang.Register;

public class ReaCreateDecomp extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        Address addr = currentAddress;
        if (args.length > 0) {
            addr = currentProgram.getAddressFactory().getDefaultAddressSpace()
                .getAddress(Long.parseLong(args[0].replace("0x",""), 16));
        }
        Function f = currentProgram.getFunctionManager().getFunctionAt(addr);
        if (f != null) {
            removeFunction(f);
            f = null;
        }
        ProgramContext ctx = currentProgram.getProgramContext();
        try {
            Register tm = ctx.getRegister("TMode");
            ctx.setValue(tm, addr, addr, BigInteger.valueOf(1));
        } catch (Exception e) {
            println("REA-RESULT: TMODE-ERR " + e);
        }
        disassemble(addr);
        f = createFunction(addr, "recovery_fn");
        if (f == null) { println("REA-RESULT: NO FUNCTION at " + addr); return; }
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        DecompileResults res = di.decompileFunction(f, 180, monitor);
        if (res.decompileCompleted()) {
            println("REA-RESULT: BEGIN " + f.getEntryPoint());
            println(res.getDecompiledFunction().getC());
            println("REA-RESULT: END");
        } else {
            println("REA-RESULT: DECOMP FAILED " + res.getErrorMessage());
        }
    }
}
